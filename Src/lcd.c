/**
 ******************************************************************************
 * @file           : lcd.c
 * @author         : Muthuu SVS
 * @brief          : 4-bit HD44780 LCD driver over SPI2 + 74HC595
 Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 ******************************************************************************
 */

#include "lcd.h"
#include "spi.h"
#include "timers.h"   
#include <stddef.h>
#include <stm32f091xc.h>
#include <stdarg.h>
#include <stdio.h>

//LCD print defines
#define LCD_COLS (16U)
#define LCD_ROWS (2U)
#define LCD_PRINTF_BUF (64U)

//LCD timing constants 
#define LCD_POWER_UP_DELAY_MS   (50U)
#define LCD_INIT_8BIT_RETRY_MS  (5U)
#define LCD_INIT_8BIT_RETRY2_MS (1U)
#define LCD_INIT_4BIT_DELAY_MS  (1U)

#define LCD_CMD_DELAY_US    (50U)
#define LCD_CLEAR_DELAY_MS  (2U)

#define LCD_NIBBLE_MASK (0x0F)

//Bit mapping from 74HC595 to LCD
/*
 * 74HC595 Outputs:
 *  Qh Qg Qf Qe | Qd Qc Qb Qa
 * DB7 DB6 DB5 DB4 |  E  RS  x  x
 */

#define LCD_DB4    (1U << 4)   // Qe
#define LCD_DB5    (1U << 5)   // Qf
#define LCD_DB6    (1U << 6)   // Qg
#define LCD_DB7    (1U << 7)   // Qh

#define LCD_E      (1U << 3)   // Qd
#define LCD_RS     (1U << 2)   // Qc

//LCD Command defines
#define LCD_CMD_CLEAR   (0x01U)
#define LCD_CMD_HOME    (0x02U)

//Entry Mode Set
#define LCD_CMD_ENTRY_MODE  (0x04U)
#define LCD_ENTRY_INCREMENT (0x02U)
#define LCD_ENTRY_DECREMENT (0x00U)
#define LCD_ENTRY_SHIFT_ON  (0x01U)
#define LCD_ENTRY_SHIFT_OFF (0x00U)

//Display Control
#define LCD_CMD_DISPLAY_CTRL    (0x08U)
#define LCD_DISPLAY_ON  (0x04U)
#define LCD_DISPLAY_OFF (0x00U)
#define LCD_CURSOR_ON   (0x02U)
#define LCD_CURSOR_OFF  (0x00U)
#define LCD_BLINK_ON    (0x01U)
#define LCD_BLINK_OFF   (0x00U)

//Function Set
#define LCD_CMD_FUNCTION_SET    (0x20U)
#define LCD_FUNCTION_8BIT   (0x10U)
#define LCD_FUNCTION_4BIT   (0x00U)
#define LCD_FUNCTION_2LINE  (0x08U)
#define LCD_FUNCTION_1LINE  (0x00U)
#define LCD_FUNCTION_5x10_DOTS  (0x04U)
#define LCD_FUNCTION_5x8_DOTS   (0x00U)

/* Set DDRAM Address */
#define LCD_CMD_SET_DDRAM          (0x80U)

/* Pulse the LCD Enable (E) line via 74HC595 output
 * Sends the provided data byte with E high, then clears E, using the
 * shift-register latch to update outputs. Small delays ensure timing
 * requirements of HD44780 controllers are met.
 * @param data_byte: byte containing DB7-DB4, RS and E mapping
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_pulse_enable(uint8_t data_byte)
{
    spi2_write(data_byte | LCD_E);
    spi2_latch();
    delay_us(1);

    spi2_write(data_byte & (uint8_t)(~LCD_E));
    spi2_latch();
    delay_us(1);
}


/* writes a nibble to the lcd screen
 * @param nibble: number being written, rs: command or data mode
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = 0;

    if (rs)
    {
        data |= LCD_RS;
    }

    data |= (uint8_t)((nibble & LCD_NIBBLE_MASK) << 4);

    spi2_write(data);
    spi2_latch();

    lcd_pulse_enable(data);
    delay_us(1);
}


/* Send a command to the LCD (high-level helper)
 * Splits a byte into two 4-bit transfers (high nibble then low nibble)
 * @param cmd: command byte
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_write_cmd(uint8_t cmd)
{
    lcd_write_nibble(cmd >> 4, 0U);
    lcd_write_nibble(cmd & LCD_NIBBLE_MASK, 0U);
    delay_us(LCD_CMD_DELAY_US);
}


/* Write a single data byte (character) to the LCD
 * @param data: character to write
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_write_data(uint8_t data)
{
    lcd_write_nibble(data >> 4, 1U);
    lcd_write_nibble(data & LCD_NIBBLE_MASK, 1U);
    delay_us(LCD_CMD_DELAY_US);
}


/* Initialize the HD44780-compatible LCD connected via 74HC595
 * Performs the startup sequence in 4-bit mode and configures display options
 * @param none
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_init(void)
{
    delay_ms(LCD_POWER_UP_DELAY_MS);

    // Force 8-bit mode 3 times
    lcd_write_nibble(0x3, 0U);
    delay_ms(LCD_INIT_8BIT_RETRY_MS);

    lcd_write_nibble(0x3, 0U);
    delay_ms(LCD_INIT_8BIT_RETRY2_MS);

    lcd_write_nibble(0x3, 0U);
    delay_ms(LCD_INIT_8BIT_RETRY2_MS);

    //Switch to 4-bit mode
    lcd_write_nibble(0x2, 0U);
    delay_ms(LCD_INIT_4BIT_DELAY_MS);

    //Function set: 4-bit, 2 lines, 5x8 font 
    lcd_write_cmd(LCD_CMD_FUNCTION_SET | LCD_FUNCTION_4BIT | LCD_FUNCTION_2LINE | LCD_FUNCTION_5x8_DOTS);

    //Display OFF
    lcd_write_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_OFF | LCD_CURSOR_OFF | LCD_BLINK_OFF);

    // Clear
    lcd_write_cmd(LCD_CMD_CLEAR);
    delay_ms(LCD_CLEAR_DELAY_MS);

    //Entry mode: increment cursor, no shift
    lcd_write_cmd(LCD_CMD_ENTRY_MODE | LCD_ENTRY_INCREMENT | LCD_ENTRY_SHIFT_OFF);

    //Display ON, cursor OFF, blink OFF
    lcd_write_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
}


/* Write a NUL-terminated string to the display at the current cursor
 * @param str: pointer to NUL-terminated C string
 * @return none
 */
void lcd_write_string(const char *str)
{
    if (!str) return;

    while (*str)
    {
        lcd_write_data((uint8_t)*str++);
    }
}


/* Set LCD cursor position
 * @param row: LCD row (0-based)
 * @param col: LCD column (0-based)
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0U) ? col : (0x40 + col);//has to be either 0/1 or 0-40
    lcd_write_cmd((uint8_t)(LCD_CMD_SET_DDRAM | addr));
}


/* Clears the LCD and resets cursor to (0,0)
 * @param none
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_clear(void)
{
    lcd_write_cmd(LCD_CMD_CLEAR);
    delay_ms(LCD_CLEAR_DELAY_MS);    // required: >1.52 ms
}


/* Moves LCD cursor to home position (0,0) without clearing RAM
 * @param none
 * @return none
 * Reference:  https://newhavendisplay.com/content/specs/NHD-0216HZ-FSW-FBW-33V3C.pdf
 */
void lcd_home(void)
{
    lcd_write_cmd(LCD_CMD_HOME);
    delay_ms(LCD_CLEAR_DELAY_MS);    // same execution time as clear
}


/* Print formatted text to the LCD with bounds checking
 * Ensures formatted output does not overflow the display
 * @param row: start row (0..LCD_ROWS-1)
 * @param col: start column (0..LCD_COLS-1)
 * @param fmt: printf-style format string
 * @return none
 */
void lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    if (row >= LCD_ROWS || col >= LCD_COLS)
    {
        return; // invalid cursor position
    }

    char buffer[LCD_PRINTF_BUF];

    /* Format the string safely */
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0)
    {
        return; // formatting error
    }

    /* Ensure we do not print past total LCD capacity */
    uint8_t cur_row = row;
    uint8_t cur_col = col;

    lcd_set_cursor(cur_row, cur_col);

    for (int i = 0; i < len; i++)
    {
        if (cur_row >= LCD_ROWS)
        {
            break;  // stop: no more rows available
        }

        lcd_write_data((uint8_t)buffer[i]);
        cur_col++;

        /* wrap to next row if needed */
        if (cur_col >= LCD_COLS)
        {
            cur_row++;
            cur_col = 0;

            if (cur_row < LCD_ROWS)
            {
                lcd_set_cursor(cur_row, cur_col);
            }
        }
    }
}
