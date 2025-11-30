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
#include <stm32f091xc.h>//device header

//LCD defines
#define LCD_SETTLE_TIME_US (50U)

// ----- Shift Register Bit Mapping -----
// Q0 → LCD D4
// Q1 → LCD D5
// Q2 → LCD D6
// Q3 → LCD D7
// Q4 → LCD EN
// Q5 → LCD RS
// Q6/Q7 unused

//shift register defines
#define LCD_D4_BIT 0
#define LCD_D5_BIT 1
#define LCD_D6_BIT 2
#define LCD_D7_BIT 3
#define LCD_EN_BIT 4
#define LCD_RS_BIT 5

//one active cs per lcd
static spi2_cs_t *lcd_sr_cs;//chip select for lcd shift register

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
    lcd_send_nibble(value >> 4, rs);
    lcd_send_nibble(value & 0x0F, rs);

    // Long delay for clear/home
    if (!rs && (value == 0x01 || value == 0x02))
    {
        delay_ms(2);
    }
}

/* writes a command byte to the lcd
 * @param uint8_t cmd, command byte to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_cmd(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
}

/* writes a data byte to the lcd
 * @param uint8_t data, data byte to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_data(uint8_t data)
{
    lcd_send_byte(data, 1);
}

/* writes a character to the lcd
 * @param char c, character to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_char(char c)
{
    lcd_write_data((uint8_t)c);
}

/* writes a string to the lcd
 * @param const char *str, pointer to null-terminated string to send
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_write_string(const char *str)
{
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
    lcd_write_cmd(0x01);
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
    lcd_write_cmd(0x02);
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
    static const uint8_t row_offsets[2] = {0x00, 0x40};
    lcd_write_cmd(0x80 | (col + row_offsets[row]));
}

/* initializes the lcd
 * @param spi2_cs_t *cs, pointer to chip select struct for lcd shift register
 * @return none
 * Reference : https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * https://github.com/omersiar/ShiftedLCD
 */
void lcd_init(spi2_cs_t *cs)
{
    lcd_sr_cs = cs;

    delay_ms(LCD_SETTLE_TIME_US); // LCD power-up

    // Initialization sequence for 4-bit mode
    lcd_send_nibble(0x03, 0);
    delay_ms(5);

    lcd_send_nibble(0x03, 0);
    delay_us(200);

    lcd_send_nibble(0x03, 0);
    delay_us(200);

    lcd_send_nibble(0x02, 0);   // Enter 4-bit mode

    // Function set: 4-bit, 2-line, 5x8 font
    lcd_write_cmd(0x28);

    // Display ON, Cursor OFF
    lcd_write_cmd(0x0C);

    // Clear
    lcd_write_cmd(0x01);
    delay_ms(2);

    // Entry mode: increment, no shift
    lcd_write_cmd(0x06);
}