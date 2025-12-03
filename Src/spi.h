/**
 ******************************************************************************
 * @file           : spi.h
 * @author         : Muthuu SVS
 * @brief          : SPI2 driver for STM32F0 with hardcoded RCLK (PB9)
 ******************************************************************************
 */

#ifndef SPI_H
#define SPI_H

#include "utilities.h"

// enable WRITE_ONLY_SPI to reduce code size if only writing to SPI devices
#define WRITE_ONLY_SPI

/* initializes SPI2 peripheral based on requested speed
 * PC3 = MOSI (AF1), PB10 = SCK (AF5)
 * also configures PC9 as RCLK latch for 74HC595
 * @param: clk_prescaler - clock prescaler value for SPI2 (0-7)
 * @return none
 */
void spi2_init(uint8_t clk_prescaler);

/* writes 1 byte of data to SPI2 peripheral
 * @param data : byte to send
 * @return none
 */
void spi2_write(uint8_t data);

/* writes multiple bytes of data to SPI2 peripheral
 * @param data   : pointer to buffer
 * @param length : number of bytes
 * @return none
 */
void spi2_write_buffer(const uint8_t *data, uint16_t length);

/* toggles PC9 latch (RCLK for 74HC595)
 * call after spi2_write() or spi2_write_buffer() to update outputs
 * @param none
 * @return none
 */
void spi2_latch(void);

#endif /* SPI_H */
