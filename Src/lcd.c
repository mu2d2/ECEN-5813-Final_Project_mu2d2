/**
 ******************************************************************************
 * @file           : lcd.c
 * @author         : Muthuu SVS
 * @brief          : 
 ******************************************************************************
 */

#include "lcd.h"
#include "log.h"
#include "timers.h"
#include <stdarg.h>//for vsnprintf
#include <stdio.h>
#include <stm32f091xc.h>//device header

//LCD defines
#define LCD_MAX_PRINTF_SIZE (64U)//max formatted string size for lcd_printf
//timing defines
#define LCD_SETTLE_TIME_US (50U)
#define LCD_POWER_ON_DELAY_MS (15U)
#define LCD_LONG_DELAY_MS (2U)

// LCD Commands
#define LCD_CMD_CLEAR            (0x01)
#define LCD_CMD_HOME             (0x02)
#define LCD_CMD_ENTRY_MODE       (0x04)
#define LCD_CMD_DISPLAY_CTRL     (0x08)
#define LCD_CMD_CURSOR_SHIFT     (0x10)
#define LCD_CMD_FUNCTION_SET     (0x20)
#define LCD_CMD_SET_CGRAM        (0x40)
#define LCD_CMD_SET_DDRAM        (0x80)

// ENTRY MODE OPTIONS
#define LCD_ENTRY_INC            (0x02)
#define LCD_ENTRY_NO_SHIFT       (0x00)

// DISPLAY CONTROL OPTIONS
#define LCD_DISPLAY_ON           (0x04)
#define LCD_CURSOR_OFF           (0x00)
#define LCD_BLINK_OFF            (0x00)

// FUNCTION SET OPTIONS
#define LCD_FS_4BIT              (0x00)
#define LCD_FS_2LINE             (0x08)
#define LCD_FS_5x8FONT           (0x00)

#define LCD_ROW1_ADDRESS      (0x00)
#define LCD_ROW2_ADDRESS      (0x40)

// ----- Shift Register Bit Mapping -----
// Q0 → LCD D4
// Q1 → LCD D5
// Q2 → LCD D6
// Q3 → LCD D7
// Q4 → LCD EN
// Q5 → LCD RS
// Q6/Q7 unused

//shift register defines
#define LCD_D4_BIT (0)
#define LCD_D5_BIT (1)
#define LCD_D6_BIT (2)
#define LCD_D7_BIT (3)
#define LCD_EN_BIT (4)
#define LCD_RS_BIT (5)


/* sets or clears a specific bit in a byte
 * @param uint8_t value, byte to modify
 * @param uint8_t bit, which bit to set or clear (0-7)
 * @param uint8_t state, 1 to set, 0 to clear 
 * @return none
 * Reference : 
 */
uint8_t set_bit(uint8_t value, uint8_t bit, uint8_t state)
{
    if(state)
    {
        value |=  (1U << bit);
    } 
    else       
    {
        value &= ~(1U << bit);
    }
    return value;
}

/* writes 8 bit raw data to the lcd shift register
 * @param none 
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 */
void lcd_shiftreg_write(uint8_t data)
{
    spi2_set_cs(0);
    spi2_write(data);
    spi2_set_cs(1);
}

/* sends a nibble (4 bits) to the lcd via the shift register
 * @param uint8_t nibble, 4 bits to send (lower nibble used)
 * @param uint8_t rs, register select: 0 = command, 1 = data
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t out = 0;

    // Put data bits on shift register
    out = set_bit(out, LCD_D4_BIT, (nibble >> 0) & 1);
    out = set_bit(out, LCD_D5_BIT, (nibble >> 1) & 1);
    out = set_bit(out, LCD_D6_BIT, (nibble >> 2) & 1);
    out = set_bit(out, LCD_D7_BIT, (nibble >> 3) & 1);

    // RS bit
    out = set_bit(out, LCD_RS_BIT, rs);

    // EN = high
    out = set_bit(out, LCD_EN_BIT, 1);
    lcd_shiftreg_write(out);//writes data   
    delay_us(1);

    // EN = low
    out = set_bit(out, LCD_EN_BIT, 0);
    lcd_shiftreg_write(out);//writes data   

    delay_us(LCD_SETTLE_TIME_US);   // LCD settle time
}

/* sends a byte (8 bits) to the lcd via the shift register
 * @param uint8_t value, 8 bits to send
 * @param uint8_t rs, register select: 0 = command, 1 = data
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_send_byte(uint8_t value, uint8_t rs)
{
    if (rs > 1)
    {
        LOG("LCD Error: RS must be 0 or 1 (got %u)\r\n", rs);
        return;
    }
    lcd_send_nibble(value >> 4, rs);
    lcd_send_nibble(value & 0x0F, rs);

    // Long delay for clear/home
    if (!rs && (value == LCD_CMD_CLEAR || value == LCD_CMD_HOME))
    {
        delay_ms(2);
    }
}

/* writes a character to the lcd
 * @param char c, character to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_char(char c)
{
    lcd_send_byte((uint8_t)c, 1);
}

/* writes a string to the lcd
 * @param const char *str, pointer to null-terminated string to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_string(const char *str)
{
    if(!str)//empty string
    {
        LOG("lcd_write_string: null pointer\r\n");
        return;
    }
    while (*str)
    {
        lcd_write_char(*str++);
    }
        
}

/* clears the lcd display
 * @param none
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_clear(void)
{
    lcd_send_byte(LCD_CMD_CLEAR, 0);
    delay_ms(2);
}


/* homes the lcd cursor
 * @param none
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_home(void)
{
    lcd_send_byte(LCD_CMD_HOME, 0);
    delay_ms(2);
}


/* sets the lcd cursor position
 * @param uint8_t col, column (0 to LCD_COLUMNS-1)
 * @param uint8_t row, row (0 to LCD_ROWS-1)
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_set_cursor(uint8_t col, uint8_t row)
{    
    if (row >= LCD_ROWS) 
    {
        row = 0; // Prevent out-of-bounds access
        LOG("lcd_set_cursor: Invalid row, resetting to 0\r\n");
    }
    if (col >= LCD_COLUMNS)
    {
        col = 0; // Prevent out-of-bounds access
        LOG("lcd_set_cursor: Invalid column, resetting to 0\r\n");
    }

    static const uint8_t row_offsets[2] = {LCD_ROW1_ADDRESS, LCD_ROW2_ADDRESS};
    lcd_send_byte(LCD_CMD_SET_DDRAM | (col + row_offsets[row]), 0);
}

/* initializes the lcd
 * @param none
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_init(void)
{

    delay_ms(LCD_POWER_ON_DELAY_MS); // LCD power-up
    LOG("power on delay over \r\n");
    // Initialization sequence for 4-bit mode
    lcd_send_nibble(0x03, 0);
    delay_ms(5);
    LOG("first ms delay \r\n");
    lcd_send_nibble(0x03, 0);
    delay_us(200);
    LOG("first delay_us in lcd init\r\n");

    lcd_send_nibble(0x03, 0);
    delay_us(200);
	LOG("Init seq done \r\n");
    lcd_send_nibble(0x02, 0);   // Enter 4-bit mode
	LOG("enter 4 bit mode \r\n");
    // Function set: 4-bit, 2-line, 5x8 font
    lcd_send_byte(LCD_CMD_FUNCTION_SET | LCD_FS_2LINE | LCD_FS_5x8FONT, 0);

    // Display ON, Cursor OFF
    lcd_send_byte(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF, 0);

    // Clear
    lcd_send_byte(LCD_CMD_CLEAR, 0);
    delay_ms(2);

    // Entry mode: increment, no shift
    lcd_send_byte(LCD_CMD_ENTRY_MODE | LCD_ENTRY_INC | LCD_ENTRY_NO_SHIFT, 0);

	LOG("entry mode increment and no shift, display on, cursor off\r\n");
}


/* prints the formatted string to the lcd
 * @param const char *format, formatted string to send
 * @return none
 * Reference : 
 * https://stackoverflow.com/questions/11302393/creating-a-customised-version-of-printf
 */
void lcd_printf(const char *format, ...)
{
    if (!format)
    {
        LOG("lcd_printf: NULL format pointer\r\n");
        return;
    }

    char buffer[LCD_MAX_PRINTF_SIZE];//creates string buffer

    va_list args;
    va_start(args, format);
    //creates string from format and args
    vsnprintf(buffer, LCD_MAX_PRINTF_SIZE, format, args);
    va_end(args);

    //moves string to lcd
    lcd_write_string(buffer);

}
