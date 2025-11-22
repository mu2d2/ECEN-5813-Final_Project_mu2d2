#include "stm32f0xx.h"  // Include the appropriate STM32 header for your MCU

// External declarations for the data and bss sections (defined in linker script)
extern uint32_t _sidata;   // Start address of the initialized data in flash
extern uint32_t _sdata;    // Start address of the .data section in SRAM
extern uint32_t _edata;    // End address of the .data section in SRAM
extern uint32_t _sbss;     // Start address of the .bss section in SRAM
extern uint32_t _ebss;     // End address of the .bss section in SRAM
extern uint32_t _estack;   // End of stack address (typically defined in the linker script)

// Function prototypes
void SystemInit(void);
void __libc_init_array(void);
void SystemCoreClockUpdate(void);
int main(void);

// Default handler for all interrupts
void Default_Handler(void)
{
    // Enter infinite loop if interrupt occurs (for debugging purposes)
    while (1);
}

// Reset Handler
void Reset_Handler(void)
{
    // Set the stack pointer to the end of the stack memory (from _estack)
    __set_MSP((uint32_t)&_estack);

    // Call system initialization function (e.g., setup clock, peripherals)
    SystemInit();

    // Copy the data segment initializers from flash to SRAM
    uint32_t *data_src = &_sidata;
    uint32_t *data_dest = &_sdata;
    while (data_dest < &_edata) {
        *data_dest++ = *data_src++;
    }

    // Zero fill the BSS segment
    uint32_t *bss_dest = &_sbss;
    while (bss_dest < &_ebss) {
        *bss_dest++ = 0;
    }

    // Call the static constructors (C runtime environment setup)
    __libc_init_array();

    // Update system core clock
    SystemCoreClockUpdate();

    // Jump to main function
    main();

    // Infinite loop if main returns (should not reach here)
    while (1);
}

// Provide weak aliases for each Exception handler to the Default_Handler
#define WEAK_ALIAS(func) \
    __attribute__((weak, alias("Default_Handler"))) void func(void);

WEAK_ALIAS(NMI_Handler)
WEAK_ALIAS(HardFault_Handler)
WEAK_ALIAS(SVC_Handler)
WEAK_ALIAS(PendSV_Handler)
WEAK_ALIAS(SysTick_Handler)
WEAK_ALIAS(WWDG_IRQHandler)
WEAK_ALIAS(PVD_IRQHandler)
WEAK_ALIAS(RTC_IRQHandler)
WEAK_ALIAS(FLASH_IRQHandler)
WEAK_ALIAS(RCC_CRS_IRQHandler)
WEAK_ALIAS(EXTI0_1_IRQHandler)
WEAK_ALIAS(EXTI2_3_IRQHandler)
WEAK_ALIAS(EXTI4_15_IRQHandler)
WEAK_ALIAS(TSC_IRQHandler)
WEAK_ALIAS(DMA1_CH1_IRQHandler)
WEAK_ALIAS(DMA1_CH2_3_DMA2_CH1_2_IRQHandler)
WEAK_ALIAS(DMA1_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler)
WEAK_ALIAS(ADC_COMP_IRQHandler)
WEAK_ALIAS(TIM1_BRK_UP_TRG_COM_IRQHandler)
WEAK_ALIAS(TIM1_CC_IRQHandler)
WEAK_ALIAS(TIM2_IRQHandler)
WEAK_ALIAS(TIM3_IRQHandler)
WEAK_ALIAS(TIM6_DAC_IRQHandler)
WEAK_ALIAS(TIM7_IRQHandler)
WEAK_ALIAS(TIM14_IRQHandler)
WEAK_ALIAS(TIM15_IRQHandler)
WEAK_ALIAS(TIM16_IRQHandler)
WEAK_ALIAS(TIM17_IRQHandler)
WEAK_ALIAS(I2C1_IRQHandler)
WEAK_ALIAS(I2C2_IRQHandler)
WEAK_ALIAS(SPI1_IRQHandler)
WEAK_ALIAS(SPI2_IRQHandler)
WEAK_ALIAS(USART1_IRQHandler)
WEAK_ALIAS(USART2_IRQHandler)
WEAK_ALIAS(USART3_4_5_6_7_8_IRQHandler)
WEAK_ALIAS(CEC_CAN_IRQHandler)
WEAK_ALIAS(USB_IRQHandler)

// Vector table
// Place the vector table at the beginning of flash memory
void * g_pfnVectors[] __attribute__((section(".isr_vector"))) =
{
    (void (*)(void))(&_estack),      // Initial Stack Pointer
    Reset_Handler,                   // Reset Handler
    NMI_Handler,                     // NMI Handler
    HardFault_Handler,               // Hard Fault Handler
    0,                               // Reserved
    0,                               // Reserved
    0,                               // Reserved
    0,                               // Reserved
    0,                               // Reserved
    0,                               // Reserved
    0,                               // Reserved
    SVC_Handler,                     // SVC Handler
    0,                               // Reserved
    0,                               // Reserved
    PendSV_Handler,                  // PendSV Handler
    SysTick_Handler,                 // SysTick Handler

    // External Interrupts
    WWDG_IRQHandler,                 // WWDG_IRQHandler
    PVD_IRQHandler,                  // PVD_IRQHandler
    RTC_IRQHandler,                  // RTC_IRQHandler
    FLASH_IRQHandler,                // FLASH_IRQHandler
    RCC_CRS_IRQHandler,              // RCC_CRS_IRQHandler
    EXTI0_1_IRQHandler,              // EXTI0_1_IRQHandler
    EXTI2_3_IRQHandler,              // EXTI2_3_IRQHandler
    EXTI4_15_IRQHandler,             // EXTI4_15_IRQHandler
    TSC_IRQHandler,                  // TSC_IRQHandler
    DMA1_CH1_IRQHandler,             // DMA1_CH1_IRQHandler
    DMA1_CH2_3_DMA2_CH1_2_IRQHandler,// DMA1_CH2_3_DMA2_CH1_2_IRQHandler
    DMA1_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler, // DMA1_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler
    ADC_COMP_IRQHandler,             // ADC_COMP_IRQHandler
    TIM1_BRK_UP_TRG_COM_IRQHandler,  // TIM1_BRK_UP_TRG_COM_IRQHandler
    TIM1_CC_IRQHandler,              // TIM1_CC_IRQHandler
    TIM2_IRQHandler,                 // TIM2_IRQHandler
    TIM3_IRQHandler,                 // TIM3_IRQHandler
    TIM6_DAC_IRQHandler,             // TIM6_DAC_IRQHandler
    TIM7_IRQHandler,                 // TIM7_IRQHandler
    TIM14_IRQHandler,                // TIM14_IRQHandler
    TIM15_IRQHandler,                // TIM15_IRQHandler
    TIM16_IRQHandler,                // TIM16_IRQHandler
    TIM17_IRQHandler,                // TIM17_IRQHandler
    I2C1_IRQHandler,                 // I2C1_IRQHandler
    I2C2_IRQHandler,                 // I2C2_IRQHandler
    SPI1_IRQHandler,                 // SPI1_IRQHandler
    SPI2_IRQHandler,                 // SPI2_IRQHandler
    USART1_IRQHandler,               // USART1_IRQHandler
    USART2_IRQHandler,               // USART2_IRQHandler
    USART3_4_5_6_7_8_IRQHandler,     // USART3_4_5_6_7_8_IRQHandler
    CEC_CAN_IRQHandler,              // CEC_CAN_IRQHandler
    USB_IRQHandler,                  // USB_IRQHandler
};

