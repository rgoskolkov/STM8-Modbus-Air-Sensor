/**
 * @file    main.c
 * @brief   STM8 Modbus Air Sensor - AHT30 (I2C) + ENS160 (SPI) + Modbus RTU
 * 
 * @note    Подключение:
 *          AHT30 (I2C):
 *            - PB4 = SCL (Open-Drain, pull-up 4.7k к 3.3V)
 *            - PB5 = SDA (Open-Drain, pull-up 4.7k к 3.3V)
 * 
 *          ENS160 (SPI):
 *            - PA3 = CS (Chip Select, активен LOW)
 *            - PC5 = SCK
 *            - PC6 = MOSI
 *            - PC7 = MISO
 * 
 *          UART (RS485):
 *            - PD5 = TX
 *            - PD6 = RX
 * 
 * @note    Modbus карта регистров (holding registers):
 *          Регистр 0: Температура (AHT30) ×10, °C
 *          Регистр 1: Влажность (AHT30) ×10, %
 *          Регистр 2: eCO2 (ENS160), ppm
 *          Регистр 3: TVOC (ENS160), ppb
 *          Регистр 4: AQI (ENS160), 1-5
 *          Регистр 5: Статус AHT30 (0=OK, иначе код ошибки)
 *          Регистр 6: PART_ID ENS160 (должен быть 0x0160)
 *          Регистр 7: DATA_STATUS ENS160
 */

#include "stm8s_conf.h"
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define BAUD_RATE           9600        /* Скорость UART */
#define SENSOR_INTERVAL     30000       /* Интервал опроса датчиков, мс */
#define UART_BUF_SIZE       16          /* Размер UART буфера */

/* Кнопка и светодиод */
#define BTN_PORT            GPIOA
#define BTN_PIN             GPIO_PIN_1
#define LED_PORT            GPIOA
#define LED_PIN             GPIO_PIN_2

/* EEPROM адреса */
#define EEPROM_ADDR         ((uint16_t)0x4000)  /* Базовый адрес EEPROM STM8 */
#define EEPROM_MODBUS_ADDR  ((uint16_t)0x4000)  /* Адрес хранения Modbus адреса */
#define EEPROM_MAGIC        0xA5                /* Магическое число (запись произведена) */
#define EEPROM_MAGIC_ADDR   ((uint16_t)0x4001)  /* Адрес магического числа */

/* Режимы работы */
#define MODE_NORMAL         0
#define MODE_CONFIG         1

/* Тайминги кнопок */
#define BTN_HOLD_MS         3000                /* Удержание кнопки для входа в режим */
#define BTN_SHORT_MS        500                 /* Короткое нажатие - инкремент */
#define BTN_MED_MS          1500                /* Среднее нажатие - декремент */
#define BTN_DEBOUNCE_MS     50                  /* Антидребезг кнопки, мс */

/* ENS160 SPI: PA3 = CS */
#define ENS160_CS_PORT      GPIOA
#define ENS160_CS_PIN       GPIO_PIN_3

/* ENS160 регистры */
#define ENS160_PART_ID_REG  0x00
#define ENS160_OPMODE_REG   0x10
#define ENS160_DATA_STATUS  0x20
#define ENS160_DATA_AQI     0x21
#define ENS160_DATA_TVOC    0x22
#define ENS160_DATA_ECO2    0x24

#define ENS160_STANDARD_MODE  0x02
#define ENS160_PART_ID        0x0160

/* TIM4 тик = 500 мкс */
#define TICK_PERIOD_US      500

/* Modbus RTU: 3.5 символа = 3.5 × 11 бит */
/* Таймаут в мкс = (3.5 × 11 × 1_000_000) / baud_rate */
#define MODBUS_TIMEOUT_US   (38500000UL / BAUD_RATE)

/* Таймаут в тиках TIM4 (округление вверх) */
#define MODBUS_TIMEOUT_TICKS ((MODBUS_TIMEOUT_US + TICK_PERIOD_US - 1) / TICK_PERIOD_US)

/* Variables -----------------------------------------------------------------*/
volatile uint16_t mb_regs[8];           /* Modbus holding registers */
volatile uint8_t uart_buf[UART_BUF_SIZE];  /* UART приёмный буфер */
volatile uint8_t uart_idx = 0;          /* Индекс UART буфера */
volatile uint8_t modbus_frame_ready = 0;  /* Флаг готовности Modbus фрейма */
volatile uint16_t line_silence_time = 0;  /* Таймер тишины линии */
volatile uint32_t _millis = 0;          /* Счётчик тиков TIM4 (500 мкс) */
volatile uint32_t sensor_timer = 0;     /* Таймер опроса датчиков */
volatile uint8_t ens160_status = 0;     /* Статус ENS160 */

/* Конфигурация */
volatile uint8_t g_modbus_addr = 1;     /* Modbus адрес (по умолчанию 1) */
volatile uint8_t g_work_mode = MODE_NORMAL;  /* Режим работы */
volatile uint32_t btn_press_time = 0;   /* Время нажатия */
volatile uint32_t btn_release_time = 0; /* Время отпускания */
volatile uint8_t btn_released = 0;      /* Флаг: кнопку отпустили */
volatile uint32_t last_btn_irq = 0;     /* Для антидребезга */
volatile uint8_t btn_locked = 0;        /* Блокировка обработки кнопки */

/* Мигание на TIM2 */
volatile uint8_t blink_cnt = 0;       /* Сколько раз мигнуть */
volatile uint8_t blink_cur = 0;       /* Текущий счёт мигания */

/**
 * @brief Запустить мигание светодиодом
 */
static void start_blink(uint8_t cnt, uint16_t period_ms) {
    uint16_t arr_val = (uint16_t)period_ms * 125;  /* 125kHz = 8мкс на тик */
    blink_cnt = cnt > 20 ? 20 : cnt;
    blink_cur = 0;
    /* TIM2: 16MHz / 128 = 125kHz = 8мкс на тик */
    TIM2->CR1 = 0;          /* Stop timer */
    TIM2->PSCR = 0x07;      /* Prescaler 128 (2^7) */
    TIM2->ARRH = (arr_val >> 8) & 0xFF;
    TIM2->ARRL = arr_val & 0xFF;
    TIM2->CNTRH = 0;
    TIM2->CNTRL = 0;
    TIM2->SR1 = 0;          /* Clear flags */
    TIM2->EGR = 0x01;       /* Generate update event */
    TIM2->IER = 0x01;       /* Update interrupt enable */
    TIM2->CR1 = 0x01;       /* Enable */
    
    /* Ждём окончания мигания */
    while (blink_cnt > 0);
}

/*====================== Задержки ===========================================*/

/**
 * @brief Задержка в миллисекундах (на основе TIM4)
 * @param ms Задержка в миллисекундах
 */
static void delay_ms(uint16_t ms) {
    uint32_t start = _millis;
    uint32_t ticks = (uint32_t)ms * (1000 / TICK_PERIOD_US);
    while ((_millis - start) < ticks);
}

/*====================== CRC16 (Modbus) =====================================*/

/**
 * @brief Расчёт CRC16 (Modbus polynomial 0xA001)
 * @param buf Буфер данных
 * @param len Длина буфера
 * @return CRC16 значение
 */
static uint16_t crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/*====================== EEPROM =============================================*/

/**
 * @brief Запись байта в EEPROM
 */
static void eeprom_write_byte(uint16_t addr, uint8_t data) {
    volatile uint16_t timeout = 20000;
    
    /* Отключаем прерывания */
    __asm__("sim");
    
    /* Разблокировка записи в EEPROM */
    FLASH->DUKR = 0xAE;
    FLASH->DUKR = 0x56;
    
    /* Запись данных */
    *(volatile uint8_t *)addr = data;
    
    /* Ждём окончания записи с таймаутом */
    while (!(FLASH->IAPSR & FLASH_IAPSR_EOP) && timeout > 0) {
        timeout--;
    }
    
    /* Блокировка записи */
    FLASH->IAPSR |= FLASH_IAPSR_EOP;
    
    /* Включаем прерывания */
    __asm__("rim");
}

/**
 * @brief Чтение байта из EEPROM
 */
static uint8_t eeprom_read_byte(uint16_t addr) {
    return *(volatile uint8_t *)addr;
}

/**
 * @brief Сохранение Modbus адреса в EEPROM
 */
static void save_modbus_addr(uint8_t addr) {
    eeprom_write_byte(EEPROM_MODBUS_ADDR, addr);
    eeprom_write_byte(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
}

/**
 * @brief Загрузка Modbus адреса из EEPROM
 * @return 1 = успех, 0 = EEPROM пуст
 */
static uint8_t load_modbus_addr(void) {
    if (eeprom_read_byte(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC) {
        g_modbus_addr = eeprom_read_byte(EEPROM_MODBUS_ADDR);
        return 1;
    }
    return 0;
}

/*====================== Кнопка и светодиод =================================*/

/**
 * @brief Включить/выключить светодиод
 */
static void led_on(void) {
    GPIO_WriteLow(LED_PORT, LED_PIN);  /* Активный низкий уровень */
}

static void led_off(void) {
    GPIO_WriteHigh(LED_PORT, LED_PIN);
}

/*====================== Режим конфигурации =================================*/

/**
 * @brief Режим конфигурации Modbus адреса
 * Короткое нажатие (<500мс) - инкремент
 * Среднее нажатие (500-1500мс) - декремент
 * Длинное нажатие - сохранить и выйти
 */
static void run_config_mode(void) {
    uint8_t new_addr = g_modbus_addr;
    uint32_t hold_time;
    /* Мигнём 2 раза - вход в режим */
    start_blink(2, 300);
    /* Сброс флагов */
    btn_released = 0;

    while (1) {
        /* Ждём ОТПУСКАНИЯ кнопки */
        if (btn_released) {
            btn_locked = 1;
            btn_released = 0;
            hold_time = btn_release_time - btn_press_time;
            if (hold_time < BTN_SHORT_MS) {
                /* Короткое - инкремент */
                new_addr++;
                if (new_addr > 247) new_addr = 1;
            } else if (hold_time < BTN_MED_MS) {
                /* Среднее - декремент */
                if (new_addr > 1) new_addr--;
                else new_addr = 247;
            } else {
                /* Длинное - сохранить и выйти */
                save_modbus_addr(new_addr);
                start_blink(3, 70); //новый адрес сохранен
                g_modbus_addr = new_addr;
                g_work_mode = MODE_NORMAL;
            }
            
            if (g_work_mode != MODE_NORMAL) {
                /* Моргнём новым номером адреса */
                uint8_t blinks = (new_addr > 20) ? 20 : new_addr;
                start_blink(blinks, 250);
                btn_locked = 0;
            } else {
                btn_locked = 0;
                return;  /* Выход из режима конфигурации */
            }
        }
    }
}

/*====================== Modbus RTU =========================================*/

/**
 * @brief Обработка Modbus запроса (функция 0x03 - Read Holding Registers)
 */
static void modbus_process(void) {
    uint8_t tx[25];
    uint16_t crc;
    uint8_t len, i;
    uint16_t reg_count;

    if (uart_buf[0] != g_modbus_addr) return;

    /* Проверка CRC */
    crc = crc16(uart_buf, uart_idx);
    if (crc != 0) return;

    if (uart_buf[1] != 0x03) return;

    reg_count = ((uint16_t)uart_buf[4] << 8) | uart_buf[5];
    len = (reg_count > 8) ? 8 : reg_count;

    tx[0] = uart_buf[0];
    tx[1] = 0x03;
    tx[2] = len * 2;

    for (i = 0; i < len; i++) {
        tx[3 + i*2] = mb_regs[i] >> 8;
        tx[4 + i*2] = mb_regs[i] & 0xFF;
    }

    crc = crc16(tx, 3 + len*2);
    tx[3 + len*2] = crc & 0xFF;
    tx[4 + len*2] = crc >> 8;

    uint8_t tx_len = 3 + len*2 + 2;
    for (i = 0; i < tx_len; i++) {
        UART1_SendData8(tx[i]);
        while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    }
    while (UART1_GetFlagStatus(UART1_FLAG_TC) == RESET);
}

/*====================== AHT30 (I2C) ========================================*/

/**
 * @brief Чтение данных с датчика AHT30
 * @return 1 = успех, 0 = ошибка
 */
static uint8_t read_aht_i2c(void) {
    uint8_t cmd[] = {0xAC, 0x33, 0x00};
    uint8_t rx[6];
    uint32_t raw_t, raw_h;
    uint8_t i;
    volatile uint32_t timeout;

    /* START + запись команды измерения */
    I2C_GenerateSTART(ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && --timeout);
    if (!timeout) { mb_regs[5] = 0x0001; return 0; }

    I2C_Send7bitAddress(0x70, I2C_DIRECTION_TX);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --timeout);
    if (!timeout) { I2C_GenerateSTOP(ENABLE); mb_regs[5] = 0x0002; return 0; }

    for (i = 0; i < 3; i++) {
        I2C_SendData(cmd[i]);
        timeout = 10000;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --timeout);
        if (!timeout) { I2C_GenerateSTOP(ENABLE); mb_regs[5] = 0x0003 + i; return 0; }
    }

    I2C_GenerateSTOP(ENABLE);
    delay_ms(80);  /* Время измерения AHT30 */

    /* Повторный START + чтение данных */
    I2C_GenerateSTART(ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && --timeout);
    if (!timeout) { mb_regs[5] = 0x0010; return 0; }

    I2C_Send7bitAddress(0x70, I2C_DIRECTION_RX);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --timeout);
    if (!timeout) { I2C_GenerateSTOP(ENABLE); mb_regs[5] = 0x0011; return 0; }

    I2C_AcknowledgeConfig(ENABLE);

    for (i = 0; i < 6; i++) {
        timeout = 10000;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) && --timeout);
        if (!timeout) { I2C_GenerateSTOP(ENABLE); mb_regs[5] = 0x0012 + i; return 0; }
        rx[i] = I2C_ReceiveData();
    }

    I2C_AcknowledgeConfig(DISABLE);
    I2C_GenerateSTOP(ENABLE);

    /* Расчёт температуры и влажности */
    raw_h = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | (rx[3] >> 4);
    raw_t = ((uint32_t)(rx[3] & 0x0F) << 16) | ((uint32_t)rx[4] << 8) | rx[5];

    mb_regs[0] = (uint16_t)((raw_t * 250UL / 131072UL) - 500);  /* Температура ×10 */
    mb_regs[1] = (uint16_t)(raw_h * 125UL / 131072UL);          /* Влажность ×10 */
    mb_regs[5] = 0x0000;  /* Успех */

    return 1;
}

/*====================== ENS160 (SPI) =======================================*/

/**
 * @brief Управление CS пином ENS160
 */
static void ens160_cs_low(void) {
    GPIO_WriteLow(ENS160_CS_PORT, ENS160_CS_PIN);
}

static void ens160_cs_high(void) {
    GPIO_WriteHigh(ENS160_CS_PORT, ENS160_CS_PIN);
}

/**
 * @brief SPI чтение/запись одного байта
 */
static uint8_t spi_transfer(uint8_t data) {
    SPI_SendData(data);
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    return SPI_ReceiveData();
}

/**
 * @brief Запись в регистр ENS160
 * @param reg Адрес регистра
 * @param data Данные для записи
 * @param len Количество байт
 */
static void ens160_write_reg(uint8_t reg, const uint8_t *data, uint8_t len) {
    ens160_cs_low();
    spi_transfer((reg << 1) & 0xFE);  /* Бит 0 = 0 для записи */
    while (len--) {
        spi_transfer(*data++);
    }
    ens160_cs_high();
}

/**
 * @brief Чтение из регистра ENS160
 * @param reg Адрес регистра
 * @param data Буфер для чтения
 * @param len Количество байт
 * @return Количество прочитанных байт
 */
static uint8_t ens160_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t i;
    ens160_cs_low();
    spi_transfer((reg << 1) | 0x01);  /* Бит 0 = 1 для чтения */
    for (i = 0; i < len; i++) {
        data[i] = spi_transfer(0x00);
    }
    ens160_cs_high();
    return len;
}

/**
 * @brief Чтение данных с датчика ENS160
 * @return 1 = успех, 0 = ошибка
 */
static uint8_t read_ens160_spi(void) {
    uint8_t buf[4];
    uint16_t part_id;
    uint16_t eco2, tvoc;
    uint8_t aqi, status;

    /* Чтение PART_ID для проверки связи */
    ens160_read_reg(ENS160_PART_ID_REG, buf, 2);
    part_id = ((uint16_t)buf[1] << 8) | buf[0];
    
    if (part_id != ENS160_PART_ID) {
        mb_regs[6] = part_id;  /* Сохраняем для отладки */
        return 0;
    }
    mb_regs[6] = part_id;  /* PART_ID = 0x0160 */

    /* Чтение статуса */
    ens160_read_reg(ENS160_DATA_STATUS, &status, 1);
    mb_regs[7] = status;
    ens160_status = status;

    /* Проверка статуса: бит 0 = новые данные, бит 1-2 = валидность */
    if ((status & 0x01) == 0) {
        /* Нет новых данных, но это не ошибка */
    }
    
    /* Проверка на ошибку (статус 3 = invalid output) */
    if ((status & 0x06) == 0x06) {
        return 0;  /* Ошибка сенсора */
    }

    /* Чтение AQI */
    ens160_read_reg(ENS160_DATA_AQI, &aqi, 1);
    
    /* Чтение TVOC (2 байта, little endian) */
    ens160_read_reg(ENS160_DATA_TVOC, buf, 2);
    tvoc = ((uint16_t)buf[1] << 8) | buf[0];
    
    /* Чтение eCO2 (2 байта, little endian) */
    ens160_read_reg(ENS160_DATA_ECO2, buf, 2);
    eco2 = ((uint16_t)buf[1] << 8) | buf[0];

    /* Запись в Modbus регистры */
    mb_regs[2] = eco2;      /* eCO2, ppm */
    mb_regs[3] = tvoc;      /* TVOC, ppb */
    mb_regs[4] = aqi;       /* AQI, 1-5 */

    return 1;
}

/**
 * @brief Инициализация ENS160
 * @return 1 = успех, 0 = ошибка
 */
static uint8_t ens160_init(void) {
    uint8_t mode = ENS160_STANDARD_MODE;
    uint16_t part_id;
    uint8_t buf[2];

    /* Задержка после подачи питания (минимум 10мс) */
    delay_ms(50);

    /* Чтение PART_ID */
    ens160_read_reg(ENS160_PART_ID_REG, buf, 2);
    part_id = ((uint16_t)buf[1] << 8) | buf[0];
    
    if (part_id != ENS160_PART_ID) {
        return 0;  /* Chip not found or wrong ID */
    }

    /* Установка стандартного режима */
    ens160_write_reg(ENS160_OPMODE_REG, &mode, 1);
    
    /* Время на инициализацию (по даташиту до 3 секунд на полный старт) */
    delay_ms(100);

    return 1;
}

/*====================== Прерывания =========================================*/

/**
 * @brief Обработчик прерывания TIM4 (500 мкс тик + Modbus таймаут)
 */
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) {
    TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
    _millis++;

    /* Modbus таймаут: 3.5 символа тишины */
    if (uart_idx > 0 && !modbus_frame_ready) {
        line_silence_time++;
        if (line_silence_time >= MODBUS_TIMEOUT_TICKS) {
            modbus_frame_ready = 1;
        }
    }
}

/**
 * @brief Обработчик прерывания TIM2 (мигание LED)
 */
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13) {
    TIM2->SR1 = 0;  /* Clear update flag */

    if (blink_cnt > 0) {
        GPIOA->ODR ^= 0x04;  /* Toggle LED */
        blink_cur++;
        if (blink_cur >= blink_cnt * 2) {  /* *2 потому что переключаем 2 раза на мигание */
            blink_cnt = 0;
            blink_cur = 0;
            TIM2->CR1 &= ~0x01;  /* Disable timer */
            GPIOA->ODR |= 0x04;  /* LED off */
        }
    }
}

/**
 * @brief Обработчик прерывания UART1 RX
 */
INTERRUPT_HANDLER(UART1_RX_TX_IRQHandler, 18) {
    if (UART1_GetITStatus(UART1_IT_RXNE) != RESET) {
        if (uart_idx < UART_BUF_SIZE) {
            uart_buf[uart_idx++] = UART1_ReceiveData8();
        }
        line_silence_time = 0;
        UART1_ClearITPendingBit(UART1_IT_RXNE);
    }
}

/**
 * @brief Обработчик прерывания EXTI (кнопка PA1)
 * Срабатывает на оба фронта
 */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3) {
    uint32_t now = _millis;

    /* Антидребезг: игнорируем если прошло < 50мс */
    if ((int32_t)(now - last_btn_irq) < BTN_DEBOUNCE_MS) {
        return;
    }
    last_btn_irq = now;

    /* Небольшая задержка для стабилизации уровня */
    for (volatile uint8_t d = 0; d < 10; d++);

    /* Проверяем уровень на пине */
    if ((GPIO_ReadInputData(BTN_PORT) & BTN_PIN) == 0) {
        /* Низкий уровень = кнопка нажата (спад) */
        if (!btn_locked) {
            btn_press_time = now;
        }
    } else {
        /* Высокий уровень = кнопка отпущена (подъём) */
        if (!btn_locked) {
            btn_release_time = now;
            btn_released = 1;
        }
    }
}


/*====================== Инициализация ======================================*/

/**
 * @brief Инициализация системы
 */
static void sys_init(void) {
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);  /* 16 MHz */

    /* UART: PD5 (TX), PD6 (RX) */
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_6);

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
    UART1_Init(BAUD_RATE, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
               UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
    UART1_Cmd(ENABLE);
    UART1_ITConfig(UART1_IT_RXNE, ENABLE);

    /* TIM4: 500 мкс тик */
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
    TIM4_TimeBaseInit(TIM4_PRESCALER_128, 62);  /* 16MHz/128/62 = 500 мкс */
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    TIM4_Cmd(ENABLE);

    /* I2C: PB4 (SCL), PB5 (SDA) */
    /* КРИТИЧНО: отключаем Schmitt Trigger для STM8S */
    GPIOB->CR2 |= (GPIO_PIN_4 | GPIO_PIN_5);
    GPIO_Init(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_SLOW);
    GPIO_WriteHigh(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
    
    /* Ручная инициализация I2C: 100 kHz */
    I2C->CR1 = 0;
    I2C->CR2 = 0x01;        /* ACK enable */
    I2C->FREQR = 16;        /* 16 MHz */
    I2C->CCRL = 80;         /* 16MHz/(100kHz*2) = 80 */
    I2C->CCRH = 0;
    I2C->TRISER = 17;       /* 16+1 для Standard mode */
    I2C->OARL = 0x00;
    I2C->OARH = 0x00;
    I2C->CR1 = I2C_CR1_PE;  /* I2C enable */

    for (volatile uint32_t d = 0; d < 10000; d++);  /* Стабилизация */

    /* SPI: PC5 = SCK, PC6 = MOSI, PC7 = MISO, PA3 = CS */
    GPIO_Init(GPIOC, GPIO_PIN_5 | GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(ENS160_CS_PORT, ENS160_CS_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_WriteHigh(ENS160_CS_PORT, ENS160_CS_PIN);  /* CS = HIGH по умолчанию */

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);

    /* Инициализация SPI: Master, Mode 0, 2 MHz, MSB first */
    /* 16MHz / 8 = 2MHz */
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_8, SPI_MODE_MASTER,
             SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE,
             SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x00);
    SPI_Cmd(ENABLE);

    /* Кнопка: PA1 = input с pull-up и прерыванием */
    GPIO_Init(BTN_PORT, BTN_PIN, GPIO_MODE_IN_PU_IT);

    /* EXTI на оба фронта для кнопки (PA1) */
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL);

    /* Светодиод: PA2 (output push-pull) */
    GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteHigh(LED_PORT, LED_PIN);  /* LED выключен (активный низкий) */

    /* TIM2 для мигания LED - включаем тактирование напрямую */
    CLK->PCKENR1 |= 0x20;  /* TIM2 clock enable, bit 5 */

    enableInterrupts();
}

/*====================== Main ===============================================*/

int main(void) {
    uint8_t i;
    uint32_t sensor_ticks;

    sys_init();
    delay_ms(100);

    for (i = 0; i < 8; i++) mb_regs[i] = 0;

    /* Загрузка Modbus адреса из EEPROM */
    if (!load_modbus_addr()) {
        /* EEPROM пуст - входим в режим конфигурации */
        g_work_mode = MODE_CONFIG;
    } else {
        /* Мигаем количеством раз, равным адресу модбас (для индикации) */
        start_blink((g_modbus_addr > 20) ? 20 : g_modbus_addr, 250);
    }
    /* Расчёт интервала опроса в тиках */
    sensor_ticks = (uint32_t)SENSOR_INTERVAL * (1000 / TICK_PERIOD_US);
    sensor_timer = _millis + sensor_ticks;

    /* Если режим конфигурации - запускаем его */
    if (g_work_mode == MODE_CONFIG) {
        run_config_mode();
    }

    /* Первичное чтение датчиков */
    read_aht_i2c();

    /* Инициализация и чтение ENS160 */
    if (ens160_init()) {
        read_ens160_spi();
    }

    while (1) {
        /* Вход в режим конфигурации при отпускании кнопки */
        if (btn_released && g_work_mode == MODE_NORMAL) {
            btn_locked = 1;
            btn_released = 0;
            uint32_t hold_time = btn_release_time - btn_press_time;

            if (hold_time >= BTN_HOLD_MS) {
                g_work_mode = MODE_CONFIG;
                run_config_mode();
                btn_locked = 0;  /* Разблокируем после выхода */
            }
        }

        /* Обработка Modbus запросов */
        if (modbus_frame_ready) {
            modbus_process();
            uart_idx = 0;
            line_silence_time = 0;
            modbus_frame_ready = 0;
        }

        /* Опрос датчиков по интервалу */
        if ((int32_t)_millis - (int32_t)sensor_timer >= 0) {
            read_aht_i2c();
            read_ens160_spi();
            sensor_timer = _millis + sensor_ticks;
        }

        delay_ms(2);
    }
}
