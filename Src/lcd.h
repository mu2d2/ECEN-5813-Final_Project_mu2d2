/**
 ******************************************************************************
 * @file           : lcd.h
 * @author         : Muthuu SVS
 * @brief          : public driver to control HD44780 compatible LCDs
 * references     : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 ******************************************************************************
 */

#ifndef LCD_H
#define LCD_H
#include "spi.h"//spi2 control and chip select struct
#include <stdint.h>

//LCD dimensions
#define LCD_COLUMNS (16U)
#define LCD_ROWS (2U)

//to-do functions
void lcd_init(spi2_cs_t *cs);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_write_char(char c);
void lcd_write_string(const char *str);
void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(uint8_t data);



#endif /* LCD_H_ */