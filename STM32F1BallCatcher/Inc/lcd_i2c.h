/*
 * lcd_i2c.h
 *
 *  Created on: May 15, 2018
 *      Author: saogalde
 *      https://github.com/afiskon/stm32-i2c-lcd-1602
 */
#ifndef LCD_I2C_H_
#define LCD_I2C_H_
#include <string.h>
#include "stdint.h"
#include "stm32f1xx_hal.h"
#define LCD_ADDR (0x27 << 1)

void LCD_SendString(uint8_t lcd_addr, char *str);
void LCDInit(I2C_HandleTypeDef* hi2c);
void LCD_SendLines(char* s1, char* s2);

#endif /* LCD_I2C_H_ */
