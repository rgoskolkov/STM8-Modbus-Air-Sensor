/**
 * @file    stm8s_conf.h
 * @brief   STM8 Standard Peripheral Library configuration
 */

#ifndef __STM8S_CONF_H__
#define __STM8S_CONF_H__

/* Включаем stm8s.h ПЕРВЫМ - он определяет bool */
#include "stm8s.h"

/* Модули библиотеки */
#include "stm8s_clk.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_spi.h"
#include "stm8s_uart1.h"
#include "stm8s_tim4.h"
#include "stm8s_exti.h"
/* TIM2, FLASH - без SPL, прямая работа с регистрами */

#define assert_param(expr) ((void)0)

#endif
