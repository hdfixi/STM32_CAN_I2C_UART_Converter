/*
 * lcd_display_i2c.h
 *
 *  Created on: Jan 22, 2024
 *      Author: DELL
 */

#ifndef INC_LCD_DISPLAY_I2C_H_
#define INC_LCD_DISPLAY_I2C_H_

#include "stm32f4xx_hal.h"

#define SLAVE_ADDRESS_LCD 0x4E


void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_init (void);
void lcd_send_string (char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);

#endif /* INC_LCD_DISPLAY_I2C_H_ */
