#include "led.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "tlc59116.h"

#include "board_config.h"

#define BLOCK_TIME 10

static const char* TAG = "led";

static struct led_s {
    gpio_num_t gpio;
    bool on : 1;
    uint16_t ontime, offtime;
    TimerHandle_t timer;
} leds[3], tlc[1];




void led_init(void)
{
    for (int i = 0; i < LED_ID_MAX; i++) {
        leds[i].timer = NULL;
        leds[i].gpio = GPIO_NUM_NC;
    }
    tlc[0].timer = NULL;

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = 0,
    };

    if (board_config.leds.wifi_gpio != -1) {
        leds[LED_ID_WIFI].gpio = board_config.leds.wifi_gpio;
        io_conf.pin_bit_mask |= BIT64(board_config.leds.wifi_gpio);
    }

    if (board_config.leds.charging_gpio != -1) {
        leds[LED_ID_CHARGING].gpio = board_config.leds.charging_gpio;
        io_conf.pin_bit_mask |= BIT64(board_config.leds.charging_gpio);
    }

    if (board_config.leds.error_gpio != -1) {
        leds[LED_ID_ERROR].gpio = board_config.leds.error_gpio;
        io_conf.pin_bit_mask |= BIT64(board_config.leds.error_gpio);
    }

    if (io_conf.pin_bit_mask > 0) {
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
}

static void timer_callback(TimerHandle_t xTimer)
{
    struct led_s* led = (struct led_s*)pvTimerGetTimerID(xTimer);

    led->on = !led->on;
    gpio_set_level(led->gpio, led->on);

    xTimerChangePeriod(xTimer, pdMS_TO_TICKS(led->on ? led->ontime : led->offtime), BLOCK_TIME);
}
static void timer_callback_tlc(TimerHandle_t xTimer)
{
    
    struct led_s* tlcc = (struct led_s*)pvTimerGetTimerID(xTimer);

    tlcc->on = !tlcc->on;
    ////gpio_set_level(tlc->gpio, tlc->on);

    //xTimerChangePeriod(xTimer, pdMS_TO_TICKS(tlcc->on ? tlcc->ontime : tlcc->offtime), BLOCK_TIME);
    //ESP_LOGI(TAG, "TLC Timer Callback");
}

void led_set_state(led_id_t led_id, uint16_t ontime, uint16_t offtime)
{
    static int tlc_running = 0; // 1=ev connected blinking, 2=ev charging solid, 3=ev not connected off

    struct led_s* led = &leds[led_id];
    struct led_s* tlcc = &tlc[0];
    if (led->gpio != GPIO_NUM_NC) {
        if (led->timer != NULL) {
            xTimerStop(led->timer, BLOCK_TIME);
            led->timer = NULL;
        }

        led->ontime = ontime;
        led->offtime = offtime;

        if (ontime == 0) {
            ESP_LOGD(TAG, "Set led %d off", led_id);
            led->on = false;
            gpio_set_level(led->gpio, led->on);
        } else if (offtime == 0) {
            ESP_LOGD(TAG, "Set led %d on", led_id);
            led->on = true;
            gpio_set_level(led->gpio, led->on);
        } else {
            ESP_LOGD(TAG, "Set led %d blink (on: %d off: %d)", led_id, ontime, offtime);

            led->on = true;
            gpio_set_level(led->gpio, led->on);

            if (led->timer == NULL) {
                led->timer = xTimerCreate("led_timer", pdMS_TO_TICKS(ontime), pdFALSE, (void*)led, timer_callback);
            }
            xTimerStart(led->timer, BLOCK_TIME);
        }
//Turn the TLC on / off based on charge led
    }else if(led_id ==  LED_ID_CHARGING && tlc_get_state() == 1){
        ESP_LOGI(TAG, "++++++++++++++++++++++++++++++++++++++++++++++++++EV status tlcrunning: %d ledid: %d:ontime %d offtime: %d", tlc_running, led_id, ontime, offtime);
        if (ontime != 0 && offtime != 0 && tlc_running != 1){
            if (tlcc->timer != NULL) {
                xTimerStop(tlcc->timer, BLOCK_TIME);
                tlcc->timer = NULL;
            }
            tlc_running = 1;
            tlcc->on = true;
            //gpio_set_level(led->gpio, led->on);

            if (tlcc->timer == NULL) {
                //tlcc->timer = xTimerCreate("tlc_timer", pdMS_TO_TICKS(ontime), pdFALSE, (void*)tlcc, timer_callback_tlc);
            }
            xTimerStart(tlcc->timer, BLOCK_TIME);
            ESP_LOGI(TAG, "EV connected ontime: %d offtime: %d", ontime, offtime);
        }else if(ontime == 1 && offtime == 0 && tlc_running != 2){
        tlc_running = 2;        
        ESP_LOGI(TAG, "EV charging! ontime: %d offtime: %d", ontime, offtime);
        }else if(ontime == 0 && offtime == 0 && tlc_running != 3){
        tlc_running = 3;
        ESP_LOGI(TAG, "EV not connected! ontime: %d offtime: %d", ontime, offtime);
        }
    }
}
