/*
  bluetooth_le.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Bluetooth comms for ESP32-S3, work in progress, not yet functional

  Part of grblHAL

  Copyright (c) 2018-2026 Terje Io

  Some parts of the code is based on example code by Espressif, in the public domain

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if BLUETOOTH_ENABLE == 1

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "bluetooth.h"
#include "grbl/grbl.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"

#define SPP_RUNNING      (1 << 0)
#define SPP_CONNECTED    (1 << 1)
#define SPP_CONGESTED    (1 << 2)
#define SPP_DISCONNECTED (1 << 3)
#define BT_TX_QUEUE_ENTRIES 32
#define BT_TX_BUFFER_SIZE 250

#define USE_BT_MUTEX 0
//#define SUPPORT_HEARTBEAT

#if USE_BT_MUTEX
#define BT_MUTEX_LOCK()   do {} while (xSemaphoreTake(lock, portMAX_DELAY) != pdPASS)
#define BT_MUTEX_UNLOCK() xSemaphoreGive(lock)
static SemaphoreHandle_t lock = NULL;
#else
#define BT_MUTEX_LOCK()
#define BT_MUTEX_UNLOCK()
#endif

#define SPP_TAG "BLUETOOTH"

typedef struct {
    volatile uint16_t head;
    char data[BT_TX_BUFFER_SIZE];
} bt_tx_buffer_t;

typedef struct {
    uint16_t length;
    uint8_t data[1];
} tx_chunk_t;

static const io_stream_t *btStreamOpen (uint32_t baud_rate);
static enqueue_realtime_command_ptr BTSetRtHandler (enqueue_realtime_command_ptr handler);
static const io_stream_status_t *get_status (uint8_t instance);

static bool is_up = false;
static uint16_t connection = 0xFFFF;
static bluetooth_settings_t bluetooth;
static SemaphoreHandle_t tx_busy = NULL;
static EventGroupHandle_t event_group = NULL;
static TaskHandle_t polltask = NULL;
static xQueueHandle tx_queue = NULL;
static portMUX_TYPE tx_flush_mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint16_t channel;
    uint8_t status;
    serial_linestate_t linestate;
    const io_stream_t *stream;
    char client_mac[18];
} session = {0};

#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_CHAR,
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SPP_SVC_INST_ID             0
/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT         0xABF5
#endif

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R','V', 'E', 'R'
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static xQueueHandle cmd_cmd_queue = NULL;

#ifdef SUPPORT_HEARTBEAT
static xQueueHandle cmd_heartbeat_queue = NULL;
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler (esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                       =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
    //SPP -  Heart beat characteristic Declaration
    [SPP_IDX_SPP_HEARTBEAT_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    //SPP -  Heart beat characteristic Value
    [SPP_IDX_SPP_HEARTBEAT_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

    //SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_HEARTBEAT_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static bt_tx_buffer_t txbuffer;
static stream_rx_buffer_t rxbuffer = {0};
static nvs_address_t nvs_address;
static io_stream_properties_t bt_streams[] = {
    {
      .type = StreamType_Bluetooth,
      .instance = 20,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = Off,
      .claim = btStreamOpen,
      .get_status = get_status
    }
};

static io_stream_status_t stream_status = {
    .baud_rate = 115200,
    .format = {
        .width = Serial_8bit,
        .stopbits = Serial_StopBits1,
        .parity = Serial_ParityNone,
    }
};

static on_report_options_ptr on_report_options;
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static enqueue_realtime_command_ptr BTSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

uint32_t BTStreamAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t BTStreamRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

int32_t BTStreamGetC (void)
{
    BT_MUTEX_LOCK();

    int32_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {
        BT_MUTEX_UNLOCK();
        return -1; // no data available else EOF
    }

    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    BT_MUTEX_UNLOCK();

    return data;
}

static inline bool enqueue_tx_chunk (uint16_t length, uint8_t *data)
{
    tx_chunk_t *chunk;

    if((chunk = malloc(sizeof(tx_chunk_t) + length))) {
        chunk->length = length;
        memcpy(&chunk->data, data, length);
        if (xQueueSendToBack(tx_queue, &chunk, portMAX_DELAY) != pdPASS) {
            free(chunk);
            chunk = NULL;
        }
    }

    return chunk != NULL;
}

// Since grblHAL always sends cr/lf terminated strings we can send complete strings to improve throughput
bool BTStreamPutC (const uint8_t c)
{
    if(txbuffer.head < BT_TX_BUFFER_SIZE)
        txbuffer.data[txbuffer.head++] = c;

    if(c == ASCII_LF) {
        enqueue_tx_chunk(txbuffer.head, (uint8_t *)txbuffer.data);
        txbuffer.head = 0;
    }

    return true;
}

void BTStreamWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        BTStreamPutC(c);
}

void BTStreamFlush (void)
{
    BT_MUTEX_LOCK();

    rxbuffer.tail = rxbuffer.head;

    BT_MUTEX_UNLOCK();
}

IRAM_ATTR void BTStreamCancel (void)
{
    BT_MUTEX_LOCK();

    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);

    BT_MUTEX_UNLOCK();
}

char *bluetooth_get_device_mac (void)
{
    static char device_mac[18];

    if(is_up) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        sprintf(device_mac, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else
        strcpy(device_mac, "-");

    return device_mac;
}

char *bluetooth_get_client_mac (void)
{
    return session.client_mac[0] == '\0' ? NULL : session.client_mac;
}

static void report_bt_MAC (bool newopt)
{
    char *client_mac;

    on_report_options(newopt);

    if(newopt)
        hal.stream.write(",BT");
    else {
        hal.stream.write("[BT DEVICE MAC:");
        hal.stream.write(bluetooth_get_device_mac());
        hal.stream.write("]" ASCII_EOL);

        if((client_mac = bluetooth_get_client_mac())) {
            hal.stream.write("[BT CLIENT MAC:");
            hal.stream.write(client_mac);
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

static void flush_tx_queue (void)
{
    if(uxQueueMessagesWaiting(tx_queue)) {

        tx_chunk_t *chunk;

        portENTER_CRITICAL(&tx_flush_mux);

        while(uxQueueMessagesWaiting(tx_queue)) {
            if(xQueueReceive(tx_queue, &chunk, (TickType_t)0) == pdTRUE)
                free(chunk);
        }

        portEXIT_CRITICAL(&tx_flush_mux);
    }
}

static const io_stream_status_t *get_status (uint8_t instance)
{
    stream_status.flags = bt_streams[0].flags;

    return &stream_status;
}

static bool is_connected (void)
{
    return session.linestate.dtr;
}

static const io_stream_t *btStreamOpen (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Bluetooth,
        .is_connected = is_connected,
        .read = BTStreamGetC,
        .write = BTStreamWriteS,
        .write_char = BTStreamPutC,
        .get_rx_buffer_free = BTStreamRXFree,
        .reset_read_buffer = BTStreamFlush,
        .cancel_read_buffer = BTStreamCancel,
        .set_enqueue_rt_handler = BTSetRtHandler
    };

    if(bt_streams[0].flags.claimed)
        return NULL;

    if(baud_rate != 0)
        bt_streams[0].flags.claimed = On;

    session.stream = &stream;

    return &stream;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {

        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
            }
            break;

        default:
            break;
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++) {
        if(handle == spp_handle_table[i]) {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer (esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL) {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0) {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    } else {
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer (void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer (void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
//        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

static void gatts_profile_event_handler (esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    ESP_LOGE(GATTS_TABLE_TAG, "!event = %d\n",event);

    switch (event) {

        case ESP_GATTS_REG_EVT:
            ESP_LOGE(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
            esp_ble_gap_set_device_name(bluetooth.device_name);

            ESP_LOGE(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
            esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

            ESP_LOGE(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
            esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
            break;

        case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
            break;

        case ESP_GATTS_WRITE_EVT: {
            res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                ESP_LOGE(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = NULL;
                    spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                    if(spp_cmd_buff == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                        break;
                    }
                    memset(spp_cmd_buff,0x0,(spp_mtu_size - 3));
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
                    xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
#ifdef SUPPORT_HEARTBEAT
                else if(res == SPP_IDX_SPP_HEARTBEAT_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = false;
                    }
                }else if(res == SPP_IDX_SPP_HEARTBEAT_VAL){
                    if((p_data->write.len == sizeof(heartbeat_s))&&(memcmp(heartbeat_s,p_data->write.value,sizeof(heartbeat_s)) == 0)){
                        heartbeat_count_num = 0;
                    }
                }
#endif
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
#ifndef SPP_DEBUG_MODE
                    esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),p_data->write.len);
#else
//                    uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
                    /*
                                 char c;
            while(size) {
                c = (char)*packet++;
                // discard input if MPG has taken over...
                if(hal.stream.type != StreamType_MPG) {

                    if(!enqueue_realtime_command(c)) {

                        uint_fast16_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

                        if(bptr == rxbuffer.tail)               // If buffer full
                            rxbuffer.overflow = 1;              // flag overflow,
                        else {
                            rxbuffer.data[rxbuffer.head] = c;   // else add data to buffer
                            rxbuffer.head = bptr;               // and update pointer
                        }
                    }
                }
                size--;
            }
                     */
#endif
                }else{
                    //TODO:
                }
            } else if((p_data->write.is_prep == true) && (res == SPP_IDX_SPP_DATA_RECV_VAL)) {
                ESP_LOGE(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
                store_wr_buffer(p_data);
            }
            break;
        }

        case ESP_GATTS_EXEC_WRITE_EVT:{
            ESP_LOGE(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
            if(p_data->exec_write.exec_write_flag){
                print_write_buffer();
                free_write_buffer();
            }
            break;
        }

        case ESP_GATTS_MTU_EVT:
            spp_mtu_size = p_data->mtu.mtu;
            break;

        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;

        case ESP_GATTS_CONNECT_EVT:
            spp_conn_id = p_data->connect.conn_id;
            spp_gatts_if = gatts_if;

            ESP_LOGE(GATTS_TABLE_TAG, "handle = %d\n",spp_conn_id);
            uint8_t *mac = p_data->connect.remote_bda;
            if(eTaskGetState(polltask) == eSuspended)
                vTaskResume(polltask);

//            sprintf(client_mac, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            /*
            if(connection == 0xFFFF) {
                connection = spp_conn_id; //param->open.handle;
                txbuffer.head = 0;
                bt_streams[0].flags.connected = stream_connect(btStreamOpen(0));

                if(eTaskGetState(polltask) == eSuspended)
                    vTaskResume(polltask);
                ESP_LOGE(GATTS_TABLE_TAG, "xhandle = %d\n", spp_conn_id);

       //         hal.stream.write_all("[MSG:BT OK]\r\n");
            } */
            session.linestate.dtr = On;
#ifdef SUPPORT_HEARTBEAT
            uint16_t cmd = 0;
            xQueueSend(cmd_heartbeat_queue,&cmd,10/portTICK_PERIOD_MS);
#endif
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            connection = 0xFFFF;
            session.client_mac[0] = '\0';
            flush_tx_queue();
            session.linestate.dtr = Off;
            if(session.stream)
                stream_disconnect(session.stream);
            session.stream = NULL;
            enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
            enable_heart_ntf = false;
            heartbeat_count_num = 0;
#endif
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;

        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            ESP_LOGE(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
            }
            else {
                memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;
        }

        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGE(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


static void pollTX (void * arg)
{
    tx_chunk_t *chunk = NULL;
    bool data_sent = false;
    uint8_t total_num = 0, current_num = 0;

    while(true) {

        if(connection != 0xFFFF) {

            data_sent = false;

//            ESP_LOGE("polls", "TX : h = %d\n", chunk->length);
continue;
            if(xQueueReceive(tx_queue, (void *)chunk, (portTickType)portMAX_DELAY) && chunk) {

                ESP_LOGE("poll", "ESP_GATTS_TX : handle = %d\n", chunk->length);

                continue;


                uint8_t *ntf_value_p = NULL;

#ifdef SUPPORT_HEARTBEAT
                if(!enable_heart_ntf){
                    ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable heartbeat Notify\n", __func__);
                    continue;
                }
#endif
                if(!enable_data_ntf){
                    ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__);
                    continue;
                }

                if(chunk->length <= (spp_mtu_size - 3)){
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], chunk->length, chunk->data, false);
                } else if(chunk->length > (spp_mtu_size - 3)) {

                    current_num = 1;
                    total_num = chunk->length / (spp_mtu_size - 7) + (chunk->length % (spp_mtu_size - 7)) ? 1 : 0;

                    if((ntf_value_p = (uint8_t *)malloc((spp_mtu_size - 3))) == NULL) {
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.2 failed\n", __func__);
                        free(chunk->data);
                        continue;
                    }

                    while(current_num <= total_num) {

                        ntf_value_p[0] = ntf_value_p[1] = '#';
                        ntf_value_p[2] = total_num;
                        ntf_value_p[3] = current_num;

                        if(current_num < total_num) {
                            memcpy(ntf_value_p + 4, chunk->data + (current_num - 1) * (spp_mtu_size - 7), spp_mtu_size - 7);
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], spp_mtu_size - 3, ntf_value_p, false);
                        } else if(current_num == total_num) {
                            memcpy(ntf_value_p + 4, chunk->data + (current_num - 1) * (spp_mtu_size - 7), (chunk->length - (current_num - 1) * (spp_mtu_size - 7)));
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], chunk->length - (current_num - 1) * (spp_mtu_size - 7) + 4, ntf_value_p, false);
                        }

                        vTaskDelay(20 / portTICK_PERIOD_MS);
                        current_num++;
                    }
                    free(ntf_value_p);
                }
                free(chunk->data);
            }

//            if(!data_sent)
//               xSemaphoreGive(tx_busy);
        }

        if(connection != 0xFFFF)
            vTaskDelay(pdMS_TO_TICKS(10));
        else
            vTaskSuspend(NULL);
    }
}

bool bluetooth_start_local (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(bt_streams) / sizeof(io_stream_properties_t),
        .streams = bt_streams,
    };

    session.client_mac[0] = '\0';

    if(bluetooth.device_name[0] == '\0' || !(event_group || (event_group = xEventGroupCreate())))
        return false;

    xEventGroupClearBits(event_group, 0xFFFFFF);
    xEventGroupSetBits(event_group, SPP_CONGESTED);

#if USE_BT_MUTEX
    if(!(lock || (lock = xSemaphoreCreateMutex())))
        return false;
#endif

    if(!(tx_queue || (tx_queue = xQueueCreate(BT_TX_QUEUE_ENTRIES, sizeof(tx_chunk_t *)))))
        return false;

    if(!(tx_busy || (tx_busy = xSemaphoreCreateBinary())))
        return false;

    xSemaphoreGive(tx_busy);

    if(polltask == NULL) {
        if(xTaskCreatePinnedToCore(pollTX, "btTX", 4096, NULL, 2, &polltask, 1) == pdPASS)
            vTaskSuspend(polltask);
        else
            return false;
    }

//    if(esp_bt_controller_get_status() == ESP_BLUEDROID_STATUS_ENABLED)
//        return true;

    if(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) == ESP_OK) {

        esp_err_t ret;

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bluedroid_init()) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        esp_ble_gatts_register_callback(gatts_event_handler);

        if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_ble_gatts_app_register(ESP_SPP_APP_ID)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }


        stream_register_streams(&streams);

        is_up = true;
    }

    return is_up;
}

bool bluetooth_disable_local (void)
{
    /*
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {

        if(connection)
            esp_spp_disconnect(connection);

        esp_spp_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();

        if (esp_bt_controller_disable() != ESP_OK)
            return false;

        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);

        if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {

            if (esp_bt_controller_deinit() != ESP_OK)
                return false;

            vTaskDelay(1);

            if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE)
                return false;
        }

        if(polltask) {
            flush_tx_queue();
            vTaskDelete(polltask);
            vEventGroupDelete(event_group);
            vSemaphoreDelete(tx_busy);
            vQueueDelete(tx_queue);
            polltask = event_group = tx_busy = tx_queue = NULL;
#if USE_BT_MUTEX
            vSemaphoreDelete(lock);
            lock = NULL;
#endif
        }
    }

    is_up = false;
*/
    return true;
}

static const setting_group_detail_t bluetooth_groups [] = {
    { Group_Root, Group_Bluetooth, "Bluetooth"},
};

static const setting_detail_t bluetooth_settings[] = {
    { Setting_BlueToothDeviceName, Group_Bluetooth, "Bluetooth device name", NULL, Format_String, "x(32)", NULL, "32", Setting_NonCore, bluetooth.device_name, NULL, NULL },
    { Setting_BlueToothServiceName, Group_Bluetooth, "Bluetooth service name", NULL, Format_String, "x(32)", NULL, "32", Setting_NonCore, bluetooth.service_name, NULL, NULL }
};

static const setting_descr_t bluetooth_settings_descr[] = {
    { Setting_BlueToothDeviceName, "Bluetooth device name." },
    { Setting_BlueToothServiceName, "Bluetooth service name." },
};

PROGMEM static const status_detail_t status_detail[] = {
   { Status_BTInitError, "Bluetooth initalisation failed." }
};

static error_details_t error_details = {
    .errors = status_detail,
    .n_errors = sizeof(status_detail) / sizeof(status_detail_t)
};

static void bluetooth_settings_restore (void)
{
    strcpy(bluetooth.device_name, BLUETOOTH_DEVICE);
    strcpy(bluetooth.service_name, BLUETOOTH_SERVICE);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&bluetooth, sizeof(bluetooth_settings_t), true);
}

static void bluetooth_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&bluetooth, nvs_address, sizeof(bluetooth_settings_t), true) != NVS_TransferResult_OK)
        bluetooth_settings_restore();
}

static void bluetooth_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&bluetooth, sizeof(bluetooth_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = bluetooth_groups,
    .n_groups = sizeof(bluetooth_groups) / sizeof(setting_group_detail_t),
    .settings = bluetooth_settings,
    .n_settings = sizeof(bluetooth_settings) / sizeof(setting_detail_t),
    .descriptions = bluetooth_settings_descr,
    .n_descriptions = sizeof(bluetooth_settings_descr) / sizeof(setting_descr_t),
    .save = bluetooth_settings_save,
    .load = bluetooth_settings_load,
    .restore = bluetooth_settings_restore
};

bool bluetooth_init_local (void)
{
    if((nvs_address = nvs_alloc(sizeof(bluetooth_settings_t)))) {

        hal.driver_cap.bluetooth = On;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_bt_MAC;

        errors_register(&error_details);
        settings_register(&setting_details);
    }

    return nvs_address != 0;
}

#endif // BLUETOOTH_ENABLE
