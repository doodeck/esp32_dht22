/* esp32_dht22.c */

// gpio specific
#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "test_dht.h"

// dht library specific
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "DHT.h"
static const char *TAG = "DHT";

/* http specific */
// #include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"

#include "esp_http_client.h"
#include <time.h>


#define GPIO_INPUT_IO_0     23
#define GPIO_INPUT_IO_1     22

const int gpio_inputs[] = {
    GPIO_INPUT_IO_0,
    GPIO_INPUT_IO_1
};
const int gpio_input_cnt = sizeof(gpio_inputs) / sizeof (gpio_inputs[0]);

// -- wait at least 2 sec before reading again ------------
const int gpio_pull_interval = 180000; // milliseconds

const int gpio_retry_cnt = 1; // when read error, typically "E (324578) DHT: Sensor Timeout" errors, retry number of times
const int gpio_retry_delay = 5000; // milliseconds between retries

static xQueueHandle dht_evt_queue = NULL;
typedef struct {
    // TODO: synchronize with NTP first int timestamp; // Unix timestamp
    int gpio_input;
    float humidity;
    float temperature;
} dht_evt_t;

static void DHT_task(void *pvParameter)
{
    setDHTgpio(4);
    ESP_LOGI(TAG, "Starting DHT Task\n\n");

    while (1)
    {
        ESP_LOGI(TAG, "=== Reading DHT ===\n");
        int ret = readDHT();

        errorHandler(ret);

        ESP_LOGI(TAG, "Hum: %.1f Tmp: %.1f\n", getHumidity(), getTemperature());

        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

static void DHT_task_queue(void *pvParameter)
{
    int counter = 0;
    ESP_LOGI(TAG, "Starting DHT Queue Task\n\n");

    while (1)
    {
        int gpio_input = gpio_inputs[counter % gpio_input_cnt];
        setDHTgpio(gpio_input);

        int ret, retry = 0;
        do {
            ESP_LOGI(TAG, "=== [Re]reading DHT (%d) ===\n", gpio_input);
            ret = readDHT();
            // time_t seconds = time(NULL);

            errorHandler(ret);
            if (DHT_OK != ret) {
                vTaskDelay(gpio_retry_delay / portTICK_RATE_MS);
            }
            retry++;
        } while (DHT_OK != ret && retry <= gpio_retry_cnt);

        if (DHT_OK == ret) {
            dht_evt_t io_evt;
            // io_evt.timestamp = seconds;
            io_evt.gpio_input = gpio_input;
            io_evt.humidity = getHumidity();
            io_evt.temperature = getTemperature();

            ESP_LOGI(TAG, "Input: %d, Hum: %.1f Tmp: %.1f\n", io_evt.gpio_input, io_evt.humidity, io_evt.temperature);

            xQueueSendFromISR(dht_evt_queue, &io_evt, NULL);
        }

        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(gpio_pull_interval / portTICK_RATE_MS);
        counter++;
    }
}

/* https experiments (esp-idf/examples/protocols/esp_http_client/) */
/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");
extern const char iot_dht22_herokuapp_com_root_cert_pem_start[] asm("_binary_iot_dht22_herokuapp_com_root_cert_pem_start");
extern const char iot_dht22_herokuapp_com_root_cert_pem_end[] asm("_binary_iot_dht22_herokuapp_com_root_cert_pem_end");
extern const char next_dht22_vercel_app_root_cert_pem_start[] asm("_binary_next_dht22_vercel_app_root_cert_pem_start");
extern const char next_dht22_vercel_app_root_cert_pem_end[] asm("_binary_next_dht22_vercel_app_root_cert_pem_end");

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                if (output_buffer != NULL) {
                    free(output_buffer);
                    output_buffer = NULL;
                }
                output_len = 0;
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}

static void https_with_hostname_path(void)
{
    esp_http_client_config_t config = {
        .host = "www.howsmyssl.com",
        .path = "/",
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .event_handler = _http_event_handler,
        .cert_pem = howsmyssl_com_root_cert_pem_start,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status 2 = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

static void https_backend_with_hostname_path(const char *chipId, dht_evt_t *p_dht_evt)
{
    char query[128];
    sprintf(query, "id=%s&in=%02d&hum=%.1f&temp=%.1f", chipId, p_dht_evt->gpio_input, p_dht_evt->humidity, p_dht_evt->temperature);
    ESP_LOGI(TAG, "Query: %s", query);

    esp_http_client_config_t config = {
        .host = "test-next-dht22-h50oxzplg-doodeck.vercel.app", // "next-dht22.vercel.app"
        .path = "/api/be/iot",
        .query = query,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .event_handler = _http_event_handler,
        .cert_pem = next_dht22_vercel_app_root_cert_pem_start,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status heroku = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}


static void http_test_task(void *pvParameters)
{
    https_with_hostname_path();
    // https_heroku_with_hostname_path();

    ESP_LOGI(TAG, "Finish https example");
    vTaskDelete(NULL);
}

static void http_test_task_queue(void *pvParameters)
{
    // get board's unique id (MAC address)
    uint8_t chipid[6];
    char s_chipid[18]; // need only 2 * 6 = 12
    esp_read_mac(chipid, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x",chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5]);
    sprintf(s_chipid, "%02x%02x%02x%02x%02x%02x",chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5]);

    dht_evt_t io_dht_evt;
    for(;;) {
        if(xQueueReceive(dht_evt_queue, &io_dht_evt, portMAX_DELAY)) {
            printf("Event, id: %s, input: %d, humidity: %.1f, temperature: %.1f\n",
                   s_chipid, io_dht_evt.gpio_input, io_dht_evt.humidity, io_dht_evt.temperature);
            // TODO
            https_backend_with_hostname_path(s_chipid, &io_dht_evt);
        }
    }

    // ESP_LOGI(TAG, "Finish https example");
    // vTaskDelete(NULL);
}

void app_main(void)
{
    const int generic = 4;

    switch (generic) {
        case 1:
        /* not working so far :-(
        test_dht_init();
        int cnt = 0;
        while(1) {
            printf("cnt: %d\n", cnt++);
            vTaskDelay(3000 / portTICK_RATE_MS);
            test_dht(GPIO_INPUT_IO_0);
        }
        */
            break;

        case 2:
            //Initialize NVS
            {
            esp_err_t ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
            {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
            }
            ESP_ERROR_CHECK(ret);

            esp_log_level_set("*", ESP_LOG_INFO);

            xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
            }
            break;
        
        case 3:
            {
            esp_err_t ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
            }
            ESP_ERROR_CHECK(ret);
            ESP_ERROR_CHECK(esp_netif_init());
            ESP_ERROR_CHECK(esp_event_loop_create_default());

            /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
            * Read "Establishing Wi-Fi or Ethernet Connection" section in
            * examples/protocols/README.md for more information about this function.
            */
            ESP_ERROR_CHECK(example_connect());
            ESP_LOGI(TAG, "Connected to AP, begin http example");

            xTaskCreate(&http_test_task, "http_test_task", 8192, NULL, 5, NULL);
            }
            break;

        case 4:
            {
            esp_err_t ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
            }
            ESP_ERROR_CHECK(ret);
            // esp_log_level_set("*", ESP_LOG_INFO);

            ESP_ERROR_CHECK(esp_netif_init());
            ESP_ERROR_CHECK(esp_event_loop_create_default());

            /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
            * Read "Establishing Wi-Fi or Ethernet Connection" section in
            * examples/protocols/README.md for more information about this function.
            */
            ESP_ERROR_CHECK(example_connect());
            ESP_LOGI(TAG, "Connected to AP, begin http example");

            dht_evt_queue = xQueueCreate(10, sizeof(dht_evt_t));

            xTaskCreate(&http_test_task_queue, "http_test_task_queue", 8192, NULL, 5, NULL);

            xTaskCreate(&DHT_task_queue, "DHT_task_queue", 2048, NULL, 5, NULL);
            }
            break;
    }
}