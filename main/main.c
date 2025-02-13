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
#include "comm.h"
#include "led.h"

#include "sht4x.h"


#define SHT4X_SDA_GPIO      GPIO_NUM_6  /*!< gpio number for I2C master data  */
#define SHT4X_SCL_GPIO      GPIO_NUM_7  /*!< gpio number for I2C master clock */

i2c_master_dev_handle_t sht4x_handle;

static const char *TAG = "Mist";

// Task handle to notify when slave has sent over its the address
static TaskHandle_t s_handshake_notify = NULL;

// The default sample rate, in milliseconds
static uint64_t s_sample_rate = 5000;

i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_SHT4X_I2C_NUM,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");

    return bus_handle;
}

void nvs_init() 
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void handle_sensor_query(const SensorQuery* query) 
{
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

static void handle_sensor_command(const SensorCommand* cmd) 
{
    ESP_LOGI(TAG, "Received command with ID: %lld", cmd->id);
    
    switch (cmd->which_body) {
        case SensorCommand_sleep_cycle_tag:
            ESP_LOGI(TAG, "Sleep cycle command with duration: %lld", 
                     cmd->body.sleep_cycle.sleep_time);
            break;
        case SensorCommand_sample_rate_tag:
            ESP_LOGI(TAG, "Sample rate command with rate: %lld", 
                     cmd->body.sample_rate.rate);
            if(cmd->body.sample_rate.rate < 500) {
                ESP_LOGE(TAG, "Sample rate cannot be less than 500ms");
                break;
            }
            s_sample_rate = cmd->body.sample_rate.rate;
            break;    
        default:
            ESP_LOGE(TAG, "Unknown command body type");
            break;
    }
}

// Record the timestamp when sending the request
// So when master sends back the timestamp, we can calculate the time difference to adjust the slave time.
static int64_t s_sending_timestamp = 0;

// Start time sync with master
static esp_err_t start_time_sync(const uint8_t master_mac_addr[ESP_NOW_ETH_ALEN]) 
{
    ESP_LOGI(TAG, "Requesting master timestamp... "MACSTR"", MAC2STR(master_mac_addr));
    SyncTime time_sync = SyncTime_init_default;
    size_t buffer_size = 0;
    if (!pb_get_encoded_size(&buffer_size, SyncTime_fields, &time_sync)) {
        ESP_LOGE(TAG, "Failed to get encoded size");
        return ESP_FAIL;
    }
    uint8_t buffer[buffer_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, SyncTime_fields, &time_sync)) {
        ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    s_sending_timestamp = time(NULL);
    return comm_send(buffer, buffer_size, master_mac_addr);
}

static esp_err_t send_msg_cb(const uint8_t* mac_addr, esp_now_send_status_t status) 
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        led_action();
    } else {
        led_fail();
    }
    return ESP_OK;
}

static esp_err_t recv_msg_cb(const CommTask_t* task) 
{
    pb_istream_t stream = pb_istream_from_buffer(task->buffer, task->buffer_size);
    
    // In Protocol Buffers, each field is prefixed with a tag that contains two pieces of information:
    //      1. The field number from your .proto file
    //      2. The wire type (encoding type)
    // It is encoded as the first byte, the tag: (field_number << 3) | wire_type
    uint32_t tag;
    if (!pb_decode_varint32(&stream, &tag)) {
        ESP_LOGE(TAG, "Failed to decode message type");
        return ESP_FAIL;
    }

    // Decode the actual message type
    MessageType message_type;
    if (!pb_decode_varint32(&stream, (uint32_t*)&message_type)) {
        return ESP_FAIL;
    }

    // In order to decode the message using the correct message type, we need to reset the stream. 
    // Reset stream to beginning, by setting the bytes_left to the total buffer size 
    stream.bytes_left = task->buffer_size;
    // and setting pointer to the beginning of the buffer
    stream.state = (void*)task->buffer;

    switch (message_type) {
        case MessageType_SENSOR_QUERY: {
            SensorQuery query = SensorQuery_init_default;
            if (pb_decode(&stream, SensorQuery_fields, &query)) {
                handle_sensor_query(&query);
            } else {
                ESP_LOGE(TAG, "Failed to decode SensorQuery: %s", PB_GET_ERROR(&stream));
                return false;
            }
            break;
        }
        case MessageType_SENSOR_COMMAND: {
            SensorCommand cmd = SensorCommand_init_default;
            if (pb_decode(&stream, &SensorCommand_msg, &cmd)) {
                handle_sensor_command(&cmd);
            } else {
                ESP_LOGE(TAG, "Failed to decode command: %s", PB_GET_ERROR(&stream));
                return ESP_FAIL;
            }
            break;
        }
        // Sensor will always listen to broadcast message.
        // When received master MAC address from broadcast, start the handshake
        case MessageType_SLAVERY_HANDSHAKE: {
            SlaveryHandshake handshake = SlaveryHandshake_init_default;
            if (pb_decode(&stream, SlaveryHandshake_fields, &handshake)) {             
                ESP_LOGI(TAG, "Received master MAC address: "MACSTR"", MAC2STR(handshake.master_mac_addr));
                if (!comm_is_peer_exist(handshake.master_mac_addr)) {
                    if (comm_add_peer(handshake.master_mac_addr, false) != ESP_OK) {
                        ESP_LOGE(TAG, "Add peer failed");
                        return ESP_FAIL;
                    }
                } else {
                    ESP_LOGI(TAG, "Peer already exists: "MACSTR, MAC2STR(handshake.master_mac_addr));
                }

                // Get sensor mac address
                uint8_t mac[6];
                esp_err_t ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Get MAC address failed");
                    return ESP_FAIL;
                }

                // Encode slave's address message
                // Remove master mac address, no need to send it
                handshake.has_master_mac_addr = true;
                handshake.has_slave_mac_addr = true;
                memcpy(handshake.slave_mac_addr, mac, ESP_NOW_ETH_ALEN);
                size_t buffer_size = 0;
                if (!pb_get_encoded_size(&buffer_size, SlaveryHandshake_fields, &handshake)) {
                    ESP_LOGE(TAG, "Get encoded size failed");
                    return ESP_FAIL;
                }
                uint8_t buffer[buffer_size];
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
                bool status = pb_encode(&stream, SlaveryHandshake_fields, &handshake);
                if (!status) {
                    ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
                    return ESP_FAIL;
                }

                // Send back to master
                ESP_LOGI(TAG, "Sending slave MAC address to master... "MACSTR"", MAC2STR(handshake.master_mac_addr));
                esp_err_t result = comm_send(buffer, buffer_size, handshake.master_mac_addr);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send slave MAC address");
                    return ESP_FAIL;
                }

                // Continue with time sync with master
                start_time_sync(handshake.master_mac_addr);

                // Handshake is done, notify the main task to unblock
                xTaskNotifyGive(s_handshake_notify);
            } else {
                ESP_LOGE(TAG, "Failed to decode SlaveryHandshake: %s", PB_GET_ERROR(&stream));
                return ESP_FAIL;
            }
            break;
        }
        case MessageType_SYNC_TIME: {
            SyncTime time_sync = SyncTime_init_default;
            if (pb_decode(&stream, SyncTime_fields, &time_sync)) {
                ESP_LOGI(TAG, "Received master timestamp: %lld", time_sync.master_timestamp);
                struct timeval tv;
                tv.tv_sec =  time(NULL) - s_sending_timestamp + time_sync.master_timestamp;
                tv.tv_usec = 0;
                settimeofday(&tv, NULL);
                ESP_LOGI(TAG, "Slave time is synced with master: %lld", time(NULL));
            } else {
                ESP_LOGE(TAG, "Decode SyncTime failed: %s", PB_GET_ERROR(&stream));
                return ESP_FAIL;
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Unknown message type: %d", message_type);
            return ESP_FAIL;
    }

    return ESP_OK;
}


void start_sensor() 
{
    float temperature, humidity;
    
    while(true) {
        ESP_LOGI(TAG, "Sending sensor data...");

        esp_err_t err = sht4x_start_measurement(sht4x_handle, SHT4X_CMD_READ_MEASUREMENT_HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        err = sht4x_read_measurement(sht4x_handle, &temperature, &humidity);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sensor data");
            continue;
        }

        // Dummy sensor data
        SensorQuery query = SensorQuery_init_default;
        // Specify the the correct sensor type, so buffer size can be calculated correctly
        query.which_body = SensorQuery_mist_sensor_tag;
        query.body.mist_sensor.timestamp = time(NULL);
        query.body.mist_sensor.humidity = humidity;
        query.body.mist_sensor.temperature = temperature;

        size_t buffer_size = 0;
        if (!pb_get_encoded_size(&buffer_size, SensorQuery_fields, &query)) {
            ESP_LOGE(TAG, "Failed to get encoded size");
            continue;
        }

        ESP_LOGI(TAG, "Encoded size: %d", buffer_size);

        uint8_t buffer[buffer_size];
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
        if (!pb_encode(&stream, SensorQuery_fields, &query)) {
            ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
            continue;
        }

        // Sends out the sensor data to all peers, excluding the broadcast address
        ESP_LOGI(TAG, "Sent sensor data to all peers");
        esp_err_t result = comm_send(buffer, buffer_size, NULL);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send sensor data to peer: %s", esp_err_to_name(result));
            continue;
        }

        vTaskDelay(s_sample_rate / portTICK_PERIOD_MS);
    }
}

static void init_wifi() {
    ESP_LOGI(TAG, "WiFi ESPNOW mode initialization starting...");
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // Sensor is a client, so station mode is used
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    // Enable long range mode
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
}

void app_main(void)
{
    led_blink();

    // Initialize NVS for wifi station mode
    nvs_init();
    // Initialize wifi
    init_wifi();
    // Initialize communication using wifi espnow 
    comm_init();
    comm_add_peer(COMM_BROADCAST_MAC_ADDR, false);
    comm_register_recv_msg_cb(recv_msg_cb);
    comm_register_send_msg_cb(send_msg_cb);

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


    // Store the handle of the current handshake task
    s_handshake_notify = xTaskGetCurrentTaskHandle();
    // Block until the handshake is done
    while(!ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS)) {
        ESP_LOGI(TAG, "Waiting for handshake...");
    }
    ESP_LOGI(TAG, "Handshake done");

    led_off();

    // Initialize I2C bus
    i2c_master_bus_handle_t bus_handle = i2c_bus_init(SHT4X_SDA_GPIO, SHT4X_SCL_GPIO);
    sht4x_handle = sht4x_device_create(bus_handle, SHT4X_I2C_ADDR_0, CONFIG_SHT4X_I2C_CLK_SPEED_HZ);
    ESP_LOGI(TAG, "Sensor initialization success");


    // Probe the sensor to check if it is connected to the bus with a 10ms timeout
    esp_err_t err = i2c_master_probe(bus_handle, SHT4X_I2C_ADDR_0, 200);

    if(err == ESP_OK) {
        ESP_LOGI(TAG, "SHT4X sensor found");
        xTaskCreate(start_sensor, "start_sensor", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "SHT4X sensor not found");
        sht4x_device_delete(sht4x_handle);
    }



    // Dummy main loop
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}