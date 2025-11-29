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
#include <utilities.h>

//enable WRITE_ONLY_SPI to reduce code size if only writing to SPI devices
#define WRITE_ONLY_SPI

/* initializes SPI2 peripheral 
 * PC3 = MOSI, PB10 = SCK, and MISO = PB14 if not WRITE_ONLY_SPI
 * @return none
 * Reference : 
 */
void spi2_init(void);

#ifndef WRITE_ONLY_SPI //functions used only if full duplex functionality is needed
    /*full duplex transfer, and read functions to-do*/
#endif

/* writes 1 bytes of data to SPI2 peripheral
 * @return none
 * Reference : 
 */
void spi2_write(uint8_t data);
/* writes multiple bytes of data to SPI2 peripheral
 * @return none
 * Reference : 
 */
void spi2_write_buffer(uint8_t* data, uint16_t length);

#endif /* SPI_H */