/* test_dht.c */
/* plenty of refactored pieces of  code */

/* careful NOT WORKING yet */

#include "test_dht.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
static int _pin = -1;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, value: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void test_dht_init()
{
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

}

int test_dht(int pin)
{
    _pin = pin;
    /* configure pin as output */
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pin
    io_conf.pin_bit_mask = (1ULL << _pin);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    /* force DHT to send low and wait a while */
    gpio_set_level(_pin, 0);
    vTaskDelay(3 / portTICK_RATE_MS); /* 3 ms */
    gpio_set_level(_pin, 1);
    vTaskDelay(0.025 / portTICK_RATE_MS); /* 0.025 us */

    /* reconfigure the pin as input and pull it up, connect callback */
    //interrupt on any edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pin
    io_conf.pin_bit_mask = (1ULL << _pin);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    /* connect callback */
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(_pin, gpio_isr_handler, (void*) _pin);

    return 0;
}
