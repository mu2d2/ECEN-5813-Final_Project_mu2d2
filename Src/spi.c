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

//struct for chip select control
struct spi2_cs
{
    GPIO_TypeDef* port;
    uint8_t pin;
};

//file scoped global
//instance of chip select struct
static spi2_cs_t spi2_cs_instance;
static spi2_cs_t *active_cs = &spi2_cs_instance;

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


/* initializes SPI2 peripheral 
* @param: clk_prescaler - clock prescaler value for SPI2
 * @return none
 * Reference : 
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

#ifndef WRITE_ONLY_SPI
    // PB14 = MISO (AF0)
    MODIFY_FIELD(SPI2_MISO_PORT->MODER, GPIO_MODER_MODER14, 2);   // AF mode
    MODIFY_FIELD(SPI2_MISO_PORT->AFR[1], GPIO_AFRH_AFSEL14, 0);   // AF0
#endif

    //PC3 = MOSI (AF0)
    MODIFY_FIELD(SPI2_MOSI_PORT->MODER, GPIO_MODER_MODER3, 2);    // AF mode
    MODIFY_FIELD(SPI2_MOSI_PORT->AFR[0], GPIO_AFRL_AFSEL3, 0);    // AF0

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
 * Reference : 
 */
void spi2_write(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data;

    while (!(SPI2->SR & SPI_SR_RXNE));
    (void)SPI2->DR;//clears the RX buffer
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
 * @param cs : struct containing port and pin information
 * @return none
 * Reference : 
 */
void spi2_configure_cs(spi2_cs_t *cs, void *port, uint8_t pin)
{
    cs->port = (GPIO_TypeDef *)port;
    cs->pin  = pin;
    active_cs = cs;

    //grabs the right peripheral clock for the given port
    if (cs->port == GPIOA)
    {
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    } 
    if (cs->port == GPIOB)
    {
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    } 
    if (cs->port == GPIOC)
    {
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    } 

    // Configure pin as output
    cs->port->MODER &= ~(3U << (cs->pin * 2));
    cs->port->MODER |=  (1U << (cs->pin * 2));   // 01 = output mode
    // default high
    spi2_set_cs(1);
}

/* SPI2 Chip Select control sets pin high or low of the active cs
 * @param state : 1 = HIGH, 0 = LOW
 * @return none
 * Reference : 
 */
void spi2_set_cs(uint8_t state)
{
    if (state)
    {
        // Set CS high
        active_cs->port->BSRR = (1U << active_cs->pin);
    }
    else
    {
        // Set CS low
        active_cs->port->BRR = (1U << active_cs->pin);
    }
}