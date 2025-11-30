/**
 ******************************************************************************
 * @file           : spi.c
 * @author         : Muthuu SVS
 * @brief          : 
 ******************************************************************************
 */
#include "spi.h"
#include "log.h"//debug
#include <stm32f091xc.h>//device header

/* Static CS info — only 1 device */
static GPIO_TypeDef *cs_port = NULL;
static uint8_t cs_pin = 0;

//SPI defines
#define SPI2_SCK_PIN        (10U) //PB10
#define SPI2_SCK_PORT       (GPIOB)
#define SPI2_MOSI_PIN       (3U)  //PC3
#define SPI2_MOSI_PORT      (GPIOC)
#ifndef WRITE_ONLY_SPI
    #define SPI2_MISO_PIN       (14U) //PB14
    #define SPI2_MISO_PORT      (GPIOB)
#endif
#define SPI2_MAX_PRESCALER  (7U) //max prescaler value


/* initializes SPI2 peripheral based on requested speed based on peripheral clk
 * PC3 = MOSI, PB10 = SCK, and MISO = PB14 if not WRITE_ONLY_SPI
 * @param: clk_prescaler - clock prescaler value for SPI2
 * @return none
 * Reference : Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Timers, Page No. 251
 */
void spi2_init(uint8_t clk_prescaler)
{
    if(clk_prescaler > SPI2_MAX_PRESCALER)
    {
        LOG("SPI2 Init Error: Invalid clock prescaler value\r\n");
        return;
    }
    // Enable clocks
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // PB10 = SCK (AF0)
    MODIFY_FIELD(SPI2_SCK_PORT->MODER, GPIO_MODER_MODER10, 2);   // AF mode
    MODIFY_FIELD(SPI2_SCK_PORT->AFR[1], GPIO_AFRH_AFSEL10, 0);   // AF0
    SPI2_SCK_PORT->OSPEEDR |= (3U << (SPI2_SCK_PIN * 2));        // High speed
    SPI2_SCK_PORT->PUPDR   &= ~(3U << (SPI2_SCK_PIN * 2));       // No pulls
#ifndef WRITE_ONLY_SPI
    // PB14 = MISO (AF0)
    MODIFY_FIELD(SPI2_MISO_PORT->MODER, GPIO_MODER_MODER14, 2);   // AF mode
    MODIFY_FIELD(SPI2_MISO_PORT->AFR[1], GPIO_AFRH_AFSEL14, 0);   // AF0
    SPI2_MISO_PORT->OSPEEDR |= (3U << (SPI2_MISO_PIN * 2));
    SPI2_MISO_PORT->PUPDR   &= ~(3U << (SPI2_MISO_PIN * 2));       // No pulls
#endif

    //PC3 = MOSI (AF0)
    MODIFY_FIELD(SPI2_MOSI_PORT->MODER, GPIO_MODER_MODER3, 2);    // AF mode
    MODIFY_FIELD(SPI2_MOSI_PORT->AFR[0], GPIO_AFRL_AFSEL3, 0);    // AF0
    SPI2_MOSI_PORT->OSPEEDR |= (3U << (SPI2_MOSI_PIN * 2));
    SPI2_MOSI_PORT->PUPDR   &= ~(3U << (SPI2_MOSI_PIN * 2));       // No pulls
    
    //spi2 master mode configuration
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    // Master mode
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_MSTR, 1);

    // Baud rate prescaler
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_BR, clk_prescaler);

    // Clock polarity & phase = 0 (mode 0)
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_CPOL, 0);
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_CPHA, 0);

    // Software NSS
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_SSM, 1);
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_SSI, 1);

    // Enable SPI2
    MODIFY_FIELD(SPI2->CR1, SPI_CR1_SPE, 1);
}

/* writes 1 bytes of data to SPI2 peripheral
 * @return none
 * Reference : Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Timers, Page No. 251
 */
void spi2_write(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data;

    //waits till transfer is done
    while (!(SPI2->SR & SPI_SR_BSY));

    if(SPI2->SR & SPI_SR_RXNE)
    {
    	(void)SPI2->DR;//clears the RX buffer
    }
}
/* writes multiple bytes of data to SPI2 peripheral
 * @return none
 * Reference : 
 */
void spi2_write_buffer(const uint8_t* data, uint16_t length)
{
     for (uint16_t i = 0; i < length; i++)
     {
        spi2_write(data[i]);
     }
}

/* SPI2 Chip Select control configuration
 * @param port : GPIO port for chip select
 * @param pin : GPIO pin number for chip select
 * @return none
 * Reference : Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 8 Timers, Page No. 251
 */
void spi2_configure_cs(spi2_cs_port_t port, uint8_t pin)
{

    //grabs the right peripheral clock and port
    switch (port)
    {
        case SPI2_CS_PORT_A:
            cs_port = GPIOA;
            RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
            break;

        case SPI2_CS_PORT_B:
            cs_port = GPIOB;
            RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
            break;

        case SPI2_CS_PORT_C:
            cs_port = GPIOC;
            RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
            break;
    }
    cs_pin = pin;

    // Configure pin as output
    cs_port->MODER &= ~(3U << (cs_pin * 2));
    cs_port->MODER |=  (1U << (cs_pin * 2));   // 01 = output mode
    //high speed and no pulls
    cs_port->OSPEEDR |= (3U << (cs_pin * 2));
    cs_port->PUPDR   &= ~(3U << (cs_pin * 2));

    // default high
    spi2_set_cs(1);
}

/* SPI2 Chip Select control sets pin high or low
 * @param cs the chipselect being changed
 * @param state : 1 = HIGH, 0 = LOW
 * @return none
 * Reference : 
 */
void spi2_set_cs(uint8_t state)
{
    if (state)
    {
        // Set CS high
        cs_port->BSRR = (1U << cs_pin);
    }
    else
    {
        // Set CS low
        cs_port->BRR = (1U << cs_pin);
    }
}
