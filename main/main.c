#include <time.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <time.h>
#include <sys/time.h>

#include "pb_encode.h"
#include "pb_decode.h"
#include "messages.pb.h"
#include "wireless.h"

#include "comm.h"

static const char *TAG = "Mist";

void nvs_init() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void handle_sensor_query(const SensorQuery* query) {
    switch (query->sensor_type) {
        case SensorType_MIST_SENSOR:
            // ESP_LOGI(TAG, "Received MIST_SENSOR query, timestamp: %lld", 
            //          query->body.mist_sensor.timestamp);
            ESP_LOGI(TAG, "Received MIST_SENSOR query, timestamp: %lld, humidity: %f, temperature: %f", 
                     query->body.mist_sensor.timestamp, query->body.mist_sensor.humidity, query->body.mist_sensor.temperature);
            break;
            
        case SensorType_AIR_SENSOR:
            ESP_LOGI(TAG, "Received AIR_SENSOR query, timestamp: %lld, humidity: %f, temperature: %f", 
                     query->body.air_sensor.timestamp, query->body.air_sensor.humidity, query->body.air_sensor.temperature);
            break;
            
        case SensorType_LIGHT_SENSOR:
            ESP_LOGI(TAG, "Received LIGHT_SENSOR query, timestamp: %lld, light intensity: %f", 
                     query->body.light_sensor.timestamp, query->body.light_sensor.intensity);
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown sensor type: %d", query->sensor_type);
            break;
    }
}

static void handle_command(const Command* cmd) {
    ESP_LOGI(TAG, "Received command with ID: %lld", cmd->id);
    
    switch (cmd->which_body) {
        case Command_sleep_cycle_tag:
            ESP_LOGI(TAG, "Sleep cycle command with duration: %lld", 
                     cmd->body.sleep_cycle.sleep_time);
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown command body type");
            break;
    }
}

static bool recv_msg_cb(const CommTask_t* task) {
    // For time sync
    static int64_t sending_timestamp = 0;
    
    pb_istream_t stream = pb_istream_from_buffer(task->buffer, task->buffer_size);
    
    // In Protocol Buffers, each field is prefixed with a tag that contains two pieces of information:
    //      1. The field number from your .proto file
    //      2. The wire type (encoding type)
    // It is encoded as the first byte, the tag: (field_number << 3) | wire_type
    uint32_t tag;
    if (!pb_decode_varint32(&stream, &tag)) {
        ESP_LOGE(TAG, "Failed to decode message type");
        return false; // or handle error
    }

    // Decode the actual message type
    MessageType message_type;
    if (!pb_decode_varint32(&stream, (uint32_t*)&message_type)) {
        return false; // or handle error
    }

    // In order to decode the message using the correct message type, we need to reset the stream. 
    // Reset stream to beginning, by setting the bytes_left to the total buffer size 
    stream.bytes_left = task->buffer_size;
    // and setting pointer to the beginning of the buffer
    stream.state = (void*)task->buffer;

    switch (message_type) {
        case MessageType_SENSOR_QUERY: {
            SensorQuery query = SensorQuery_init_zero;
            if (pb_decode(&stream, SensorQuery_fields, &query)) {
                handle_sensor_query(&query);
            } else {
                ESP_LOGE(TAG, "Failed to decode SensorQuery: %s", PB_GET_ERROR(&stream));
                return false;
            }
            break;
        }
        case MessageType_COMMAND: {
            Command cmd = Command_init_zero;
            if (pb_decode(&stream, &Command_msg, &cmd)) {
                handle_command(&cmd);
            } else {
                ESP_LOGE(TAG, "Failed to decode command: %s", PB_GET_ERROR(&stream));
                return false;
            }
            break;
        }
        // Received master MAC address from broadcast
        case MessageType_SLAVERY_HANDSHAKE: {
            SlaveryHandshake handshake = SlaveryHandshake_init_zero;
            handshake.message_type = MessageType_SLAVERY_HANDSHAKE;
            if (pb_decode(&stream, SlaveryHandshake_fields, &handshake)) {             
                ESP_LOGI(TAG, "Received master MAC address: "MACSTR"", MAC2STR(handshake.master_mac_addr));
                if (comm_is_peer_exist(handshake.master_mac_addr)) {
                    ESP_LOGI(TAG, "Peer already exists: "MACSTR, MAC2STR(handshake.master_mac_addr));
                    return true;
                }

                if (comm_add_peer(handshake.master_mac_addr, false) != ESP_OK) {
                    ESP_LOGE(TAG, "Add peer failed");
                    return false;
                }

                // Get sensor mac address
                uint8_t mac[6];
                esp_err_t ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Get MAC address failed");
                    return false;
                }

                // Encode slave's address message
                // Remove master mac address, no need to send it
                handshake.has_master_mac_addr = false;
                handshake.has_slave_mac_addr = true;
                memcpy(handshake.slave_mac_addr, mac, ESP_NOW_ETH_ALEN);
                size_t buffer_size = 0;
                if (!pb_get_encoded_size(&buffer_size, SlaveryHandshake_fields, &handshake)) {
                    ESP_LOGE(TAG, "Get encoded size failed");
                    return false;
                }
                uint8_t buffer[buffer_size];
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
                bool status = pb_encode(&stream, SlaveryHandshake_fields, &handshake);
                if (!status) {
                    ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
                    return false;
                }

                // Send back to master
                ESP_LOGI(TAG, "Sending slave MAC address to master...");
                esp_err_t result = comm_send(buffer, buffer_size, handshake.master_mac_addr);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send slave MAC address");
                    return false;
                }
                // Stop listening to broadcast
                comm_remove_peer(COMM_BROADCAST_MAC_ADDR);

                // Continue with syncing time with master
                ESP_LOGI(TAG, "Requesting master timestamp...");
                SyncTime time_sync = SyncTime_init_zero;
                time_sync.message_type = MessageType_SYNC_TIME;
                buffer_size = 0;
                if (!pb_get_encoded_size(&buffer_size, SyncTime_fields, &time_sync)) {
                    ESP_LOGE(TAG, "Failed to get encoded size");
                    return false;
                }
                uint8_t time_sync_buffer[buffer_size];
                stream = pb_ostream_from_buffer(time_sync_buffer, buffer_size);
                if (!pb_encode(&stream, SyncTime_fields, &time_sync)) {
                    ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
                    return false;
                }

                sending_timestamp = time(NULL);
                comm_send(time_sync_buffer, buffer_size, handshake.master_mac_addr);

            } else {
                ESP_LOGE(TAG, "Failed to decode SlaveryHandshake: %s", PB_GET_ERROR(&stream));
                return false;
            }
            break;
        }
        case MessageType_SYNC_TIME: {
            SyncTime time_sync = SyncTime_init_zero;
            time_sync.message_type = MessageType_SYNC_TIME;
            if (pb_decode(&stream, SyncTime_fields, &time_sync)) {
                ESP_LOGI(TAG, "Received master timestamp: %lld", time_sync.master_timestamp);
                struct timeval tv;
                tv.tv_sec =  time(NULL) - sending_timestamp + time_sync.master_timestamp;
                tv.tv_usec = 0;
                settimeofday(&tv, NULL);
                ESP_LOGI(TAG, "Slave time is synced with master: %lld", time(NULL));
            } else {
                ESP_LOGE(TAG, "Decode SyncTime failed: %s", PB_GET_ERROR(&stream));
                return false;
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Unknown message type: %d", message_type);
            return false;
    }

    return true;
}

void app_main(void)
{
    // Initialize NVS for wifi station mode
    nvs_init();
    // Swith WiFi to ESPNOW mode
    wl_wifi_espnow_init();
    // Initialize ESPNOW
    comm_init();
    comm_add_peer(COMM_BROADCAST_MAC_ADDR, false);
    comm_register_recv_msg_cb(recv_msg_cb);


    // Get current time
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // Print time information
    ESP_LOGI(TAG, "Current time info:");
    ESP_LOGI(TAG, "  Unix timestamp: %lld", (long long)now);
    ESP_LOGI(TAG, "  UTC time:       %s", asctime(&timeinfo));
    ESP_LOGI(TAG, "  Local time:     %s", ctime(&now));



    // Dummy main loop
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // // Broadcast sensor mac address
    // uint8_t mac[6];
    // esp_err_t ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to get MAC address");
    //     return;
    // }
    // // Broadcast the MAC address for 60 seconds
    // for(uint i = 0; i < 60; i++) {
    //     ESP_LOGI(TAG, "Broadcasting MAC address...");
    //     broadcast(mac, ESP_NOW_ETH_ALEN);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // TODO: grab the time

    // // Create a sensor query message
    // SensorQuery query = SensorQuery_init_zero;
    // query.message_type = MessageType_SENSOR_QUERY;
    // query.sensor_type = SensorType_AIR_SENSOR;
    // // If you need to set the body (optional)
    // query.which_body = SensorQuery_air_sensor_tag;
    // query.body.air_sensor.sensor_type = SensorType_AIR_SENSOR;
    // query.body.air_sensor.timestamp = time(NULL);
    // query.body.air_sensor.humidity = 0.5;
    // query.body.air_sensor.temperature = 20;
    // query.body.air_sensor.pressure = 1013.25;
    // memcpy(query.body.air_sensor.mac_addr, COMM_BROADCAST_MAC_ADDR, ESP_NOW_ETH_ALEN);

    // // Log the sensor query message
    // ESP_LOGI(TAG, "Sensor query message created:");
    // ESP_LOGI(TAG, "  Message type: %d", query.message_type);
    // ESP_LOGI(TAG, "  Sensor type:  %d", query.sensor_type);

    // // Create output buffer for encoding
    // size_t buffer_size = 0;
    // if (!pb_get_encoded_size(&buffer_size, SensorQuery_fields, &query)) {
    //     ESP_LOGE(TAG, "Failed to get encoded size");
    //     return;
    // }
    // uint8_t buffer[buffer_size];

    // // Create stream for encoding
    // pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    // // Encode the message
    // bool status = pb_encode(&stream, SensorQuery_fields, &query);
    // if (!status) {
    //     ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
    //     return;
    // }

    // // Broadcast the encoded message
    // esp_err_t result = broadcast(buffer, buffer_size);
    // if (result != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to broadcast message");
    //     return;
    // }
}