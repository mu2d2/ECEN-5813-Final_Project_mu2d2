/**
 ******************************************************************************
 * @file           : lcd.h
 * @author         : Muthuu SVS
 * @brief          : 4-bit HD44780 LCD driver over SPI2 + 74HC595
 ******************************************************************************
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

/* Initialize the HD44780-compatible LCD connected via 74HC595
 * Performs the startup sequence in 4-bit mode and configures display options
 * @param none
 * @return none
 */
void lcd_init(void);

/* Write a command byte to the LCD (RS = 0)
 * @param cmd: command byte to send
 * @return none
 */
void lcd_write_cmd(uint8_t cmd);

/* Write a data byte to the LCD (RS = 1)
 * @param data: data byte (character) to display
 * @return none
 */
void lcd_write_data(uint8_t data);

/* Write a null-terminated string to the LCD at current cursor
 * @param str: pointer to NUL-terminated string
 * @return none
 */
void lcd_write_string(const char *str);

/* Set LCD cursor position
 * @param row: LCD row (0-based)
 * @param col: LCD column (0-based)
 * @return none
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/* Clears the LCD and resets cursor to (0,0)
 * @param none
 * @return none
 */
void lcd_clear(void);

/* Moves LCD cursor to home position (0,0) without clearing RAM
 * @param none
 * @return none
 */
void lcd_home(void);

/* Formatted print into LCD (safe, bounds-checked)
 * @param row LCD row (0 or 1)
 * @param col LCD column (0-15)
 * @param fmt printf-style format string
 * @return none
 */
void lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...);


#endif /* LCD_H */
