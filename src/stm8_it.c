/**
 * @file    stm8_it.c
 * @brief   Interrupt service routines - ПУСТЫЕ, всё в main.c
 */

#include "stm8s_conf.h"
#include "stm8s.h"

/* Все обработчики в main.c - здесь заглушки */
INTERRUPT_HANDLER(NonHandledInterrupt, 0) {}
INTERRUPT_HANDLER(TRAP_IRQHandler, 1) {}
INTERRUPT_HANDLER(TLI_IRQHandler, 2) {}
INTERRUPT_HANDLER(AWU_IRQHandler, 3) {}
INTERRUPT_HANDLER(CLK_IRQHandler, 4) {}
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 6) {}
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 7) {}
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 8) {}
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 9) {}
INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 10) {}
/* TIM2_UPD_OVF_IRQHandler в main.c */
INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 22) {}
INTERRUPT_HANDLER(SPI_IRQHandler, 19) {}
INTERRUPT_HANDLER(I2C_IRQHandler, 20) {}
