//
// mqtt.c - MQTT client API for grblHAL, ESP32 version
//
// v0.1 / 2023-02-09 / Io Engineering / Terje
//

/*

Copyright (c) 2023, Terje Io

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

#if MQTT_ENABLE

#include <assert.h>
#include <string.h>

#include "networking/networking.h"
#include "networking/mqtt.h"

#include "mqtt_client.h"
#include "mqtt_supported_features.h"

static uint32_t retries = 0;
static bool connecting = false;
static esp_mqtt_client_handle_t client;

mqtt_events_t mqtt_events;

static bool do_connect (void);

static void event_handler_callback (void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch(event->event_id) {

        case MQTT_EVENT_CONNECTED:
            retries = 0;
            if(mqtt_events.on_client_connected)
                mqtt_events.on_client_connected(true);
            break;

        case MQTT_EVENT_DISCONNECTED:
            retries++;
            connecting = false;
            if(mqtt_events.on_client_connected)
                mqtt_events.on_client_connected(false);
            if(retries < 10) // TODO: retry after delay...
                do_connect();
            break;

        case MQTT_EVENT_DATA:
/*            if(arg != NULL)
                ((on_mqtt_message_received_ptr)arg)(mqtt_message.topic, (void *)mqtt_message.payload, mqtt_message.payload_length);
            else*/ if(mqtt_events.on_message_received)
                mqtt_events.on_message_received(event->topic, (void *)event->data, (size_t)event->topic_len);
            break;

        default:
/*            retries++;
            connecting = false;
            if(mqtt_events.on_client_connected)
                mqtt_events.on_client_connected(false);*/
            break;
    }
}

static bool do_connect (void)
{
    if(!connecting && client && esp_mqtt_client_start(client) == ESP_OK)
        connecting = true;

    return connecting;
}

bool mqtt_subscribe_topic (const char *topic, uint8_t qos, on_mqtt_message_received_ptr on_message_received)
{
    return on_message_received ? false : esp_mqtt_client_subscribe(client, topic, qos) == ESP_OK;
}

bool mqtt_unsubscribe_topic (const char *topic, on_mqtt_message_received_ptr on_message_received)
{
    return esp_mqtt_client_unsubscribe(client, topic) == ESP_OK;
}

bool mqtt_publish_message (const char *topic, const void *payload, size_t payload_length, uint8_t qos, bool retain)
{
    return esp_mqtt_client_publish(client, topic, payload, (int)payload_length, (int)qos, (int)retain) == ESP_OK;
}

static bool isnull (char *d, size_t len)
{
    do {
        if(*d++ != 0)
            return false;
    } while(--len);

    return true;
}

bool mqtt_connect (mqtt_settings_t *mqtt, const char *client_id)
{
    static char uri[INET6_ADDRSTRLEN + 7];

    if(!connecting && mqtt->port > 0 && !isnull(mqtt->ip, sizeof(mqtt->ip))) {

        esp_mqtt_client_config_t mqtt_cfg = {0};

        strcpy(uri, "mqtt://");
        inet_ntop(AF_INET, &mqtt->ip, uri + 7, INET6_ADDRSTRLEN);

        mqtt_cfg.client_id = client_id;
        mqtt_cfg.uri = uri;
        mqtt_cfg.port = mqtt->port;
        mqtt_cfg.username = mqtt->user;
        mqtt_cfg.password = mqtt->password;
//        mqtt_cfg.protocol_ver = MQTT_PROTOCOL_V_3_1_1;

        if((client = esp_mqtt_client_init(&mqtt_cfg))) {
            esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, event_handler_callback, NULL);
            connecting = do_connect();
        }
    }

    return connecting;
}

#endif
