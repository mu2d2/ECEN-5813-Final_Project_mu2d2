/**
 ******************************************************************************
 * @file           : spi.c
 * @author         : Muthuu SVS
 * @brief          : SPI2 driver with hardcoded PB9 RCLK latch (74HC595)
 ******************************************************************************
 */

#include "spi.h"
#include "log.h"
#include <stm32f091xc.h>

/* PC3 = MOSI, PB10 = SCK, PC9 = RCLK latch */
#define SPI2_SCK_PIN        (10U)          // PB10
#define SPI2_SCK_PORT       (GPIOB)

#define SPI2_MOSI_PIN       (3U)           // PC3
#define SPI2_MOSI_PORT      (GPIOC)

#define SPI2_RCLK_PIN       (9U)           // PC9
#define SPI2_RCLK_PORT      (GPIOC)

#define SPI2_MAX_PRESCALER  (7U)


/* initializes SPI2 peripheral based on requested speed
 * PC3 = MOSI (AF1), PB10 = SCK (AF5)
 * also configures PC9 as RCLK latch for 74HC595
 * @param: clk_prescaler - clock prescaler value for SPI2 (0-7)
 * @return none
 * Reference: Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Serial Communications, Page No. 249
    RM0091 Reference Manual
 */
void spi2_init(uint8_t clk_prescaler)
{
    if (clk_prescaler > SPI2_MAX_PRESCALER)
    {
        LOG("SPI2 Init Error: Invalid clock prescaler value\r\n");
        return;
    }

    /* Enable peripheral clocks */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    //Configure PB10 = SPI2_SCK (AF5)
    // MODER10 = 0b10 (alternate function)
    MODIFY_FIELD(SPI2_SCK_PORT->MODER, GPIO_MODER_MODER10, 2U);
    // AFRH10 = AF5 for SPI2_SCK
    MODIFY_FIELD(SPI2_SCK_PORT->AFR[1], GPIO_AFRH_AFSEL10, 5U);
    SPI2_SCK_PORT->OSPEEDR |= (3U << (SPI2_SCK_PIN * 2));   // high speed
    SPI2_SCK_PORT->PUPDR   &= ~(3U << (SPI2_SCK_PIN * 2));  // no pulls

    //Configure PC3 = SPI2_MOSI (AF1)
    // MODER3 = 0b10 (alternate function)
    MODIFY_FIELD(SPI2_MOSI_PORT->MODER, GPIO_MODER_MODER3, 2U);
    // AFRL3 = AF1 for SPI2_MOSI
    MODIFY_FIELD(SPI2_MOSI_PORT->AFR[0], GPIO_AFRL_AFSEL3, 1U);
    SPI2_MOSI_PORT->OSPEEDR |= (3U << (SPI2_MOSI_PIN * 2)); // high speed
    SPI2_MOSI_PORT->PUPDR   &= ~(3U << (SPI2_MOSI_PIN * 2));// no pulls

    //Configure PC9 = RCLK latch (GPIO output)
    // MODER9 = 0b01 (general purpose output)
    MODIFY_FIELD(SPI2_RCLK_PORT->MODER, GPIO_MODER_MODER9, 1U);
    SPI2_RCLK_PORT->OSPEEDR |= (3U << (SPI2_RCLK_PIN * 2)); // high speed
    SPI2_RCLK_PORT->PUPDR   &= ~(3U << (SPI2_RCLK_PIN * 2));// no pulls
    SPI2_RCLK_PORT->BRR = (1U << SPI2_RCLK_PIN);            // default LOW

    //Configure SPI2 peripheral
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    // Master mode
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_MSTR, 1U);

    // Baud rate prescaler
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_BR, clk_prescaler);

    // Clock polarity & phase = 0 (mode 0)
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_CPOL, 0U);
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_CPHA, 0U);

    // Software NSS management
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_SSM, 1U);
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_SSI, 1U);

    // Enable SPI2
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_SPE, 1U);
}


/* writes 1 byte of data to SPI2 peripheral
 * @param data : byte to send
 * @return none
 * Reference: Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Serial Communications, Page No. 249
    RM0091 Reference Manual
 */
void spi2_write(uint8_t data)
{
    // wait until TX buffer is empty
    while (!(SPI2->SR & SPI_SR_TXE))
    {
        // spin
    }

    // write data (byte access to avoid surprises)
    *(volatile uint8_t *)&SPI2->DR = data;

    // wait until SPI is no longer busy
    while (SPI2->SR & SPI_SR_BSY)
    {
        // spin
    }

    // clear RXNE by dummy read if needed
    if (SPI2->SR & SPI_SR_RXNE)
    {
        (void)SPI2->DR;
    }
}


/* writes multiple bytes of data to SPI2 peripheral
 * @param data   : pointer to buffer
 * @param length : number of bytes
 * @return none
 */
void spi2_write_buffer(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        spi2_write(data[i]);
    }
}


/* toggles PC9 latch (RCLK for 74HC595)
 * call after spi2_write() or spi2_write_buffer() to update outputs
 * @param none
 * @return none
 * Reference: https://www.ti.com/lit/ds/symlink/sn74hc595.pdf
 */
void spi2_latch(void)
{
    SPI2_RCLK_PORT->BSRR = (1U << SPI2_RCLK_PIN);    // HIGH
    __NOP();
    __NOP();// small delay
    SPI2_RCLK_PORT->BRR  = (1U << SPI2_RCLK_PIN);    // LOW
}
