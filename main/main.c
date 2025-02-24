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
#include "esp_adc/adc_oneshot.h"
#include <math.h>

#include <sht4x.h>
#include <sgp40.h>

#include "pb_encode.h"
#include "pb_decode.h"
#include "messages.pb.h"
#include "comm.h"
#include "led.h"

#define I2C_SDA_GPIO      GPIO_NUM_6  /*!< gpio number for I2C master data  */
#define I2C_SCL_GPIO      GPIO_NUM_7  /*!< gpio number for I2C master clock */

static const char *TAG = "Mist";

// Task handle to notify when slave has sent over its the address
static TaskHandle_t s_handshake_notify = NULL;

// The default sample rate, in milliseconds
static uint64_t s_sample_rate = 2000;

static adc_oneshot_unit_handle_t adc_handle;

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
                ESP_LOGE(TAG, "Sample rate cannot be less than 50ms");
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

// ADC Calibration
static bool adc_calibration(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void send_sensor_data(const void *src_struct) {
    size_t buffer_size = 0;
    if (!pb_get_encoded_size(&buffer_size, SensorData_fields, src_struct)) {
        ESP_LOGE(TAG, "Failed to get encoded size");
        return;
    }

    ESP_LOGI(TAG, "Encoded size: %d", buffer_size);

    uint8_t buffer[buffer_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, SensorData_fields, src_struct)) {
        ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
        return;
    }

    // Sends out the sensor data to all peers, excluding the broadcast address
    ESP_LOGI(TAG, "Sent sensor data to all peers");
    esp_err_t result = comm_send(buffer, buffer_size, NULL);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send sensor data to peer: %s", esp_err_to_name(result));
    }
}

static void measure_task()
{
    // setup SHT3x
    sht4x_t sht;
    memset(&sht, 0, sizeof(sht));
    ESP_ERROR_CHECK(sht4x_init_desc(&sht, 0, I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_ERROR_CHECK(sht4x_init(&sht));
    ESP_LOGI(TAG, "Humidity sensor initilalized");

    // setup SGP40
    sgp40_t sgp;
    memset(&sgp, 0, sizeof(sgp));
    ESP_ERROR_CHECK(sgp40_init_desc(&sgp, 0, I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_ERROR_CHECK(sgp40_init(&sgp));
    ESP_LOGI(TAG, "SGP40 initilalized. Serial: 0x%04x%04x%04x", sgp.serial[0], sgp.serial[1], sgp.serial[2]);

    // Start pre-heating the SGP40 sensor for awhile
    for (int32_t i=0; i<240; i++) {
        float temperature, humidity;
        ESP_ERROR_CHECK(sht4x_measure(&sht, &temperature, &humidity));
        // Feed it to SGP40
        int32_t voc_index;
        ESP_ERROR_CHECK(sgp40_measure_voc(&sgp, 0, 0, &voc_index));
        ESP_LOGI(TAG, "Preheating %.2f °C, %.2f %%, VOC index: %" PRIi32 "", temperature, humidity, voc_index);

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Start soil moisture sensor setup
    int adc_raw;
    int voltage;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &config));

    // ADC Calibration
    adc_cali_handle_t adc_cali_handle = NULL;
    bool calibrated = adc_calibration(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc_cali_handle);

    TickType_t last_wakeup = xTaskGetTickCount();
    while (1)
    {
        float temperature, humidity;
        ESP_ERROR_CHECK(sht4x_measure(&sht, &temperature, &humidity));
        // Feed it to SGP40
        int32_t voc_index;
        ESP_ERROR_CHECK(sgp40_measure_voc(&sgp, humidity, temperature, &voc_index));
        ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC index: %" PRIi32 "", temperature, humidity, voc_index);

        // Air sensor data
        SensorData airData = SensorData_init_default;
        airData.sensor_type = SensorType_AIR_SENSOR;
        // Specify the the correct sensor type, so buffer size can be calculated correctly
        airData.which_body = SensorData_air_sensor_tag;
        airData.body.air_sensor.timestamp = time(NULL);
        airData.body.air_sensor.humidity = humidity;
        airData.body.air_sensor.temperature = temperature;
        airData.body.air_sensor.voc_index = voc_index;

        send_sensor_data(&airData);

        // Measure soil moisture on specific adc channel
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw);
        float soil_moisture = 0;
        if (calibrated) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage));
            soil_moisture = fmax(fmin((voltage - 860) / 1240.0, 1.0), 0.0);
        }

        // Soil sensor data
        SensorData soilData = SensorData_init_default;
        soilData.sensor_type = SensorType_SOIL_SENSOR;
        // Specify the the correct sensor type, so buffer size can be calculated correctly
        soilData.which_body = SensorData_soil_sensor_tag;
        soilData.body.soil_sensor.timestamp = time(NULL);
        soilData.body.soil_sensor.moisture = soil_moisture;

        send_sensor_data(&soilData);

        // Wait until 1 seconds (VOC cycle time) are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(s_sample_rate));
    }
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

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(measure_task, "measure_task", 4096, NULL, 5, NULL);
}