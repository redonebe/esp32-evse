#ifndef TLC59116_H_
#define TLC59116_H_

#include <stdint.h>
#include "board_config.h"

#define tlc59116_address 0x60  /*!< Default I2C address for TLC59116 */
#define I2C_SLAVE_NUM    0
#define I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief Initialize Tesla leds
 *
 */
void tlc59116_init(void);

//static esp_err_t tlc95116_register_read(i2c_master_dev_handle_t dev_handle,
//                                        uint8_t reg_addr, uint8_t* data,
//                                        size_t len)

#endif /* TLC59116_H_ */
