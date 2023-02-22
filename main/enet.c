//
// enet.c - lwIP/FreeRTOS TCP/IP stream implementation
//

/*

Copyright (c) 2018-2023, Terje Io
Copyright (c) 2022, @Henrikastro
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "driver.h"
#include "enet.h"

#if ETHERNET_ENABLE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "networking/networking.h"

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

// some defines for old lwIP stack

#ifndef IPADDR_STRLEN_MAX
#define IPADDR_STRLEN_MAX 16
#endif

#ifndef ip4addr_ntoa_r
#define ip4addr_ntoa_r(ipaddr, buf, buflen) ipaddr_ntoa_r(ipaddr, buf, buflen)
#endif

#ifndef ip4addr_aton
#define ip4addr_aton(cp, addr) ipaddr_aton(cp, addr)
#endif

#define INPUT_GPIO_INTERRUPT 35
#define INPUT_GPIO_MISO 19
#define INPUT_GPIO_MOSI 23
#define INPUT_GPIO_CS 5
#define INPUT_GPIO_SCLK 18

//

static volatile bool linkUp = false;
static stream_type_t active_stream = StreamType_Null;
static network_settings_t network, ethernet;
static network_services_t services = {0}, allowed_services;
static uint32_t nvs_address;
static on_report_options_ptr on_report_options;
static on_stream_changed_ptr on_stream_changed;
static uint8_t mac_addr[6] = {0};
static esp_netif_ip_info_t *ip_info = NULL;

static char netservices[NETWORK_SERVICES_LEN] = "";

static char *enet_ip_address (void)
{
    static char ip[IPADDR_STRLEN_MAX];

    sprintf(ip, IPSTR, IP2STR(&ip_info->ip));

 //   ip4addr_ntoa_r((const ip_addr_t *)&ip_info->ip, ip, IPADDR_STRLEN_MAX);

    return ip;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(newopt) {
        hal.stream.write(",ETH");
#if FTP_ENABLE
        if(services.ftp)
            hal.stream.write(",FTP");
#endif
    } else {

        network_info_t *net = networking_get_info();

        hal.stream.write("[MAC:");
        hal.stream.write(net->mac);
        hal.stream.write("]" ASCII_EOL);

        hal.stream.write("[IP:");
        hal.stream.write(net->status.ip);
        hal.stream.write("]" ASCII_EOL);

        if(active_stream == StreamType_Telnet || active_stream == StreamType_WebSocket) {
            hal.stream.write("[NETCON:");
            hal.stream.write(active_stream == StreamType_Telnet ? "Telnet" : "Websocket");
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

network_info_t *networking_get_info (void)
{
    static network_info_t info;

    memcpy(&info.status, &network, sizeof(network_settings_t));

    sprintf(info.mac, MAC_FORMAT_STRING, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    if(ip_info)
        strcpy(info.status.ip, enet_ip_address());

    if(info.status.ip_mode == IpMode_DHCP) {
        *info.status.gateway = '\0';
        *info.status.mask = '\0';
    }

    info.is_ethernet = true;
    info.link_up = linkUp;
    info.mbps = 100;
    info.status.services = services;

#if MQTT_ENABLE
    networking_make_mqtt_clientid(info.mac, info.mqtt_client_id);
#endif

    return &info;
}

static void lwIPHostTimerHandler (void *arg)
{
    if(services.mask)
        sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);

#if TELNET_ENABLE
    if(services.telnet)
        telnetd_poll();
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        websocketd_poll();
#endif
#if FTP_ENABLE
    if(services.ftp)
        ftpd_poll();
#endif
}

static void start_services (void)
{
#if TELNET_ENABLE
    if(network.services.telnet && !services.telnet)
        services.telnet = telnetd_init(network.telnet_port == 0 ? NETWORK_TELNET_PORT : network.telnet_port);
#endif
#if WEBSOCKET_ENABLE
    if(network.services.websocket && !services.websocket)
        services.websocket = websocketd_init(network.websocket_port == 0 ? NETWORK_WEBSOCKET_PORT : network.websocket_port);
#endif
#if FTP_ENABLE
    if(network.services.ftp && !services.ftp)
        services.ftp = ftpd_init(network.ftp_port == 0 ? NETWORK_FTP_PORT : network.ftp_port);
#endif
#if HTTP_ENABLE
    if(network.services.http && !services.http)
        services.http = httpd_init(network.http_port);
#endif
#if TELNET_ENABLE || WEBSOCKET_ENABLE || FTP_ENABLE
    sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
#endif
}

/** Event handler for Ethernet events */
static void eth_event_handler (void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {

        case ETHERNET_EVENT_CONNECTED:
            linkUp = true;
            esp_eth_ioctl(*(esp_eth_handle_t *)event_data, ETH_CMD_G_MAC_ADDR, mac_addr);
            break;

        case ETHERNET_EVENT_DISCONNECTED:
            linkUp = false;
            ip_info = NULL;
            break;
/*
        case ETHERNET_EVENT_START:
            break;

        case ETHERNET_EVENT_STOP:
            break;
*/
        default:
            break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static esp_netif_ip_info_t info;

    memcpy(&info, &((ip_event_got_ip_t *)event_data)->ip_info, sizeof(esp_netif_ip_info_t));
    ip_info = &info;

    start_services();
}

static inline void get_addr (esp_ip4_addr_t *addr, char *ip)
{
    memcpy(addr, ip, sizeof(esp_ip4_addr_t));
}

bool enet_start (void)
{
    if(esp_netif_init() != ESP_OK)
        return false;

    esp_netif_t *eth_netif;

    memcpy(&network, &ethernet, sizeof(network_settings_t));

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if(network.ip_mode != IpMode_DHCP) {

        esp_netif_ip_info_t static_ip;
        get_addr(&static_ip.ip, network.ip);
        get_addr(&static_ip.gw, network.gateway);
        get_addr(&static_ip.netmask, network.mask);

        const esp_netif_inherent_config_t eth_behav_cfg = {
            .get_ip_event = IP_EVENT_ETH_GOT_IP,
            .lost_ip_event = 0,
            .flags = ESP_NETIF_DHCP_SERVER|ESP_NETIF_FLAG_AUTOUP,
            .ip_info = &static_ip,
            .if_key = "ETH_DHCPS",
            .if_desc = "eth",
            .route_prio = 50
        };

        esp_netif_config_t eth_as_dhcps_cfg = {
            .base = &eth_behav_cfg,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
        };
        eth_netif = esp_netif_new(&eth_as_dhcps_cfg);

    } else {
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        eth_netif = esp_netif_new(&cfg);
    }

    ESP_ERROR_CHECK(esp_eth_set_default_handlers(eth_netif));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = -1;

    spi_device_handle_t spi_handle = NULL;
    spi_bus_config_t buscfg = {
        .miso_io_num = INPUT_GPIO_MISO,
        .mosi_io_num = INPUT_GPIO_MOSI,
        .sclk_io_num = INPUT_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(1, &buscfg, 1));

    spi_device_interface_config_t devcfg = {
        .command_bits = 16,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = 23 * 1000 * 1000,
        .spics_io_num = INPUT_GPIO_CS,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(1, &devcfg, &spi_handle));

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
    w5500_config.int_gpio_num = INPUT_GPIO_INTERRUPT;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    if(esp_read_mac(mac_addr, ESP_MAC_ETH) == ESP_OK)
        esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);
    else
        esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]){ 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 });

    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    return true;
}

static void ethernet_settings_load (void);
static void ethernet_settings_restore (void);
static status_code_t ethernet_set_ip (setting_id_t setting, char *value);
static char *ethernet_get_ip (setting_id_t setting);
static status_code_t ethernet_set_services (setting_id_t setting, uint_fast16_t int_value);
static uint32_t ethernet_get_services (setting_id_t id);

static const setting_group_detail_t ethernet_groups [] = {
    { Group_Root, Group_Networking, "Networking" }
};

static const setting_detail_t ethernet_settings[] = {
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, netservices, NULL, NULL, Setting_NonCoreFn, ethernet_set_services, ethernet_get_services, NULL, { .reboot_required = On } },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, ethernet.hostname, NULL, NULL, { .reboot_required = On } },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL, Setting_NonCore, &ethernet.ip_mode, NULL, NULL, { .reboot_required = On } },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL, { .reboot_required = On } },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL, { .reboot_required = On } },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL, { .reboot_required = On } },
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.telnet_port, NULL, NULL, { .reboot_required = On } },
#if FTP_ENABLE
    { Setting_FtpPort, Group_Networking, "FTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.ftp_port, NULL, NULL, { .reboot_required = On } },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.http_port, NULL, NULL, { .reboot_required = On } },
#endif
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.websocket_port, NULL, NULL, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t ethernet_settings_descr[] = {
    { Setting_NetworkServices, "Network services to enable. Consult driver documentation for availability." },
    { Setting_Hostname, "Network hostname." },
    { Setting_IpMode, "IP Mode." },
    { Setting_IpAddress, "Static IP address." },
    { Setting_Gateway, "Static gateway address." },
    { Setting_NetMask, "Static netmask." },
    { Setting_TelnetPort, "(Raw) Telnet port number listening for incoming connections." },
#if FTP_ENABLE
    { Setting_FtpPort, "FTP port number listening for incoming connections." },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, "HTTP port number listening for incoming connections." },
#endif
    { Setting_WebSocketPort, "Websocket port number listening for incoming connections."
                             "NOTE: WebUI requires this to be HTTP port number + 1."
    }
};

#endif

static void ethernet_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = ethernet_groups,
    .n_groups = sizeof(ethernet_groups) / sizeof(setting_group_detail_t),
    .settings = ethernet_settings,
    .n_settings = sizeof(ethernet_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = ethernet_settings_descr,
    .n_descriptions = sizeof(ethernet_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = ethernet_settings_save,
    .load = ethernet_settings_load,
    .restore = ethernet_settings_restore
};

static status_code_t ethernet_set_ip (setting_id_t setting, char *value)
{
    ip_addr_t addr;

    if(ip4addr_aton(value, &addr) != 1)
        return Status_InvalidStatement;

    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_IpAddress:
            *((ip_addr_t *)ethernet.ip) = addr;
            break;

        case Setting_Gateway:
            *((ip_addr_t *)ethernet.gateway) = addr;
            break;

        case Setting_NetMask:
            *((ip_addr_t *)ethernet.mask) = addr;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static char *ethernet_get_ip (setting_id_t setting)
{
    static char ip[IPADDR_STRLEN_MAX];

    switch(setting) {

        case Setting_IpAddress:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.ip, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_Gateway:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.gateway, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_NetMask:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.mask, ip, IPADDR_STRLEN_MAX);
            break;

        default:
            *ip = '\0';
            break;
    }

    return ip;
}

static status_code_t ethernet_set_services (setting_id_t setting, uint_fast16_t int_value)
{
    ethernet.services.mask = int_value & allowed_services.mask;

    return Status_OK;
}

static uint32_t ethernet_get_services (setting_id_t id)
{
    return (uint32_t)ethernet.services.mask & allowed_services.mask;
}

static void ethernet_settings_restore (void)
{
    strcpy(ethernet.hostname, NETWORK_HOSTNAME);

    ip_addr_t addr;

    ethernet.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        *((ip_addr_t *)ethernet.ip) = addr;

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        *((ip_addr_t *)ethernet.gateway) = addr;

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        *((ip_addr_t *)ethernet.mask) = addr;
#else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        *((ip_addr_t *)ethernet.mask) = addr;
#endif

    ethernet.services.mask = 0;
    ethernet.ftp_port = NETWORK_FTP_PORT;
    ethernet.telnet_port = NETWORK_TELNET_PORT;
    ethernet.http_port = NETWORK_HTTP_PORT;
    ethernet.websocket_port = NETWORK_WEBSOCKET_PORT;
    ethernet.services.mask = allowed_services.mask;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static void ethernet_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ethernet, nvs_address, sizeof(network_settings_t), true) != NVS_TransferResult_OK)
        ethernet_settings_restore();

    ethernet.services.mask &= allowed_services.mask;
}

static void stream_changed (stream_type_t type)
{
    if(type != StreamType_SDCard)
        active_stream = type;

    if(on_stream_changed)
        on_stream_changed(type);
}

bool enet_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(network_settings_t)))) {

        hal.driver_cap.ethernet = On;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        on_stream_changed = grbl.on_stream_changed;
        grbl.on_stream_changed = stream_changed;

        settings_register(&setting_details);

        allowed_services.mask = networking_get_services_list((char *)netservices).mask;
    }

    return true;
}

#endif
