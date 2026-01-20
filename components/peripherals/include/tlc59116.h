#ifndef TLC59116_H_
#define TLC59116_H_

#include <stdint.h>
#include "board_config.h"

#define TLC_59116_ADDRESS 0x60  /*!< Default I2C address for TLC59116 */
#define I2C_SLAVE_NUM    0
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_SCL_GPIO     9
#define I2C_SDA_GPIO     8
#define TLC_PWM 0x06        // Set PWM value for LEDs
#define TLC_MODE0_REG 0x00
#define TLC_MODE1_REG 0x01
#define TLC_PWM_REG_0 0x02
#define TLC_PWM_REG_1 0x03
#define TLC_PWM_REG_2 0x04
#define TLC_PWM_REG_3 0x05
#define TLC_PWM_REG_4 0x06
#define TLC_PWM_REG_5 0x07
#define TLC_PWM_REG_6 0x08
#define TLC_PWM_REG_7 0x09
#define TLC_PWM_REG_8 0x0A
#define TLC_PWM_REG_9 0x0B
#define TLC_PWM_REG_10 0x0C
#define TLC_PWM_REG_11 0x0D
#define TLC_PWM_REG_12 0x0E
#define TLC_PWM_REG_13 0x0F
#define TLC_PWM_REG_14 0x10
#define TLC_PWM_REG_15 0x11
#define TLC_LED_REG_0 0x14
#define TLC_LED_REG_1 0x15
#define TLC_LED_REG_2 0x16
#define TLC_LED_REG_3 0x17


static int tlc_ready = -1;
/*
 * @brief Initialize Tesla leds
 *
 */
void tlc59116_init(void);

int tlc_get_state(void);

#endif /* TLC59116_H_ */
