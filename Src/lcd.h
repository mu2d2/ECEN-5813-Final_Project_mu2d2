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

/* prints the formatted string to the lcd
 * @param const char *format, formatted string to send
 * @return none
 * Reference : 
 * https://stackoverflow.com/questions/11302393/creating-a-customised-version-of-printf
 */
void lcd_printf(const char *format, ...);

/* initializes the lcd
 * @param none
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_init(void);

/* clears the lcd display
 * @param none
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_clear(void);

/* homes the lcd cursor
 * @param none
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_home(void);

/* sets the lcd cursor position
 * @param uint8_t col, column (0 to LCD_COLUMNS-1)
 * @param uint8_t row, row (0 to LCD_ROWS-1)
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_set_cursor(uint8_t col, uint8_t row);

/* writes a character to the lcd
 * @param char c, character to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_char(char c);

/* writes a string to the lcd
 * @param const char *str, pointer to null-terminated string to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_string(const char *str);


#endif /* LCD_H_ */