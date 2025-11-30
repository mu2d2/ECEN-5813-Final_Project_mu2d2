/**
 ******************************************************************************
 * @file           : spi.h
 * @author         : Muthuu SVS
 * @brief          : modular spi library that initializes and controls SPI2
 * for standard peripheral library based STM32F0 devices
 ******************************************************************************
 */

#ifndef SPI_H
#define SPI_H
#include "utilities.h"

typedef struct spi2_cs spi2_cs_t;//forward declaration struct for chip select control

//enable WRITE_ONLY_SPI to reduce code size if only writing to SPI devices
#define WRITE_ONLY_SPI

/* initializes SPI2 peripheral based on requested speed based on peripheral clk
 * PC3 = MOSI, PB10 = SCK, and MISO = PB14 if not WRITE_ONLY_SPI
 * @param: clk_prescaler - clock prescaler value for SPI2
 * @return none
 * Reference : Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Timers, Page No. 251
 */
void spi2_init(uint8_t clk_prescaler)

/* SPI2 Chip Select control configuration
 * @param cs : struct containing port and pin information
 * @return none
 * Reference : Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Timers, Page No. 251
 */
void spi2_configure_cs(spi2_cs_t cs);

/* SPI2 Chip Select control sets pin high or low
 * @param state : 1 = HIGH, 0 = LOW
 * @return none
 * Reference : 
 */
void spi2_set_cs(uint8_t state);

#ifndef WRITE_ONLY_SPI //functions used only if full duplex functionality is needed
    /*full duplex transfer, and read functions to-do*/
    uint8_t spi2_transfer(uint8_t data);
    uint8_t spi2_read(void);
#endif

/* writes 1 bytes of data to SPI2 peripheral
 * @return none
 * Reference : Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Timers, Page No. 251
 */
void spi2_write(uint8_t data);

/* writes multiple bytes of data to SPI2 peripheral
 * @return none
 * Reference : 
 */
void spi2_write_buffer(const uint8_t* data, uint16_t length);

#endif /* SPI_H */