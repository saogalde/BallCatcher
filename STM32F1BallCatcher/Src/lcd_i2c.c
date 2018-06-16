/*
 * lcd_i2c.c
 *
 *  Created on: May 15, 2018
 *      Author: saogalde
 */
#include "lcd_i2c.h"

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

I2C_HandleTypeDef* i2cdevice;

void I2C_Scan() {

    char info[] = "Scanning I2C bus...\r\n";
    //HAL_UART_Transmit(uartdevice, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    volatile HAL_StatusTypeDef res;
    for(uint16_t i = 0; i < 128; i++) {
        res = HAL_I2C_IsDeviceReady(i2cdevice, i << 1, 1, 10);
        if(res == HAL_OK) {
            char msg[64];
            snprintf(msg, sizeof(msg), "0x%02X", i);
            //HAL_UART_Transmit(uartdevice, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } else {
            //HAL_UART_Transmit(uartdevice, (uint8_t*)".", 1, HAL_MAX_DELAY);
        }
    }

    //HAL_UART_Transmit(uartdevice, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(i2cdevice, lcd_addr, 1, HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
        else{
        	//HAL_UART_Transmit(uartdevice, (uint8_t*)",", 1, HAL_MAX_DELAY);
        }
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    res = HAL_I2C_Master_Transmit(i2cdevice, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
    HAL_Delay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(lcd_addr, 0b00110000);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd_addr, 0b00000010);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, 0b00001100);
    // clear display (optional here)
    LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

void LCD_SendLines(char* s1, char* s2){
	LCD_SendCommand(LCD_ADDR, 0b00000001);
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, s1);
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, s2);
}

void LCDInit(I2C_HandleTypeDef* hi2c) {
	i2cdevice = hi2c;
	//uartdevice = ud;
    //I2C_Scan();
    LCD_Init(LCD_ADDR);
}

void loop() {
    HAL_Delay(100);
}
