

#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tlc59116.h"
static const char* TAG = "tlc59116";

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
        .scl_io_num = board_config.i2c.scl_gpio,
        .sda_io_num = board_config.i2c.sda_gpio,
        .glitch_ignore_cnt = 7,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .device_address = tlc59116_address,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
       
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, 0x14, 0b10101010)); //
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, 0x15, 0b10101010)); 
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, 0x16, 0b10101010));
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, 0x17, 0b10101010)); 
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, 0x00, 0x01)); // OSC on
        for (uint8_t i_t = 0x02; i_t < 0x12; i_t++) {
        ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, i_t, 0x00));
        }
        // for (uint8_t i_t = 0x02; i_t < 0x12; i_t++) {
        // vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(tlc_register_write_byte(dev_handle, 0x02, 0xff)); //Device ready
        // }
    ESP_LOGI(TAG, "TLC59116 initialized successfully.");        
}