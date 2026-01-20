#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tlc59116.h"
static const char* TAG = "tlc59116";

int tlc_get_state() {
    return tlc_ready;
}

// static esp_err_t tlc_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t* data, size_t len)
//  {
//     return i2c_master_transmit_receive( dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

static esp_err_t tlc_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void tlc59116_init() {

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_SLAVE_NUM,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");

    if(i2c_master_probe(bus_handle, TLC_59116_ADDRESS, 100) == 0) {
        ESP_LOGI(TAG, "TLC59116 found at address 0x%02X", TLC_59116_ADDRESS);
        tlc_ready = 1;
    } else {
        ESP_LOGE(TAG, "TLC59116 not found at address 0x%02X", TLC_59116_ADDRESS);
        return;
    }

    i2c_device_config_t dev_cfg = {
        .device_address = TLC_59116_ADDRESS,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

 
    // LDRx = 10: LED driver x is individual brightness can be controlled through its PWMx register.
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, TLC_LED_REG_0, 0b10101010)); //
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, TLC_LED_REG_1, 0b10101010)); 
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, TLC_LED_REG_2, 0b10101010));
    
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, TLC_LED_REG_3, 0b10101010)); 
    // Set MODE0 register OSC on
    
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, TLC_MODE0_REG, 0x01)); // OSC on
    // Turn LEDS OFF
        for (uint8_t i_t = 0x02; i_t < 0x12; i_t++) {
        ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, i_t, 0x00));
        }
        // for (uint8_t i_t = 0x02; i_t < 0x12; i_t++) {
        // vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, TLC_PWM_REG_0, TLC_PWM)); //Device ready
        // }
    ESP_LOGI(TAG, "TLC59116 initialized successfully.");        
}