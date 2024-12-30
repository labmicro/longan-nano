/************************************************************************************************
Copyright (c) 2022-2023, Laboratorio de Microprocesadores
Facultad de Ciencias Exactas y Tecnología, Universidad Nacional de Tucumán
https://www.microprocesadores.unt.edu.ar/

Copyright (c) 2022-2023, Esteban Volentini <evolentini@herrera.unt.edu.ar>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: MIT
*************************************************************************************************/

/** \brief Hello World sample application
 **
 ** \addtogroup samples Samples
 ** \brief Samples applications with MUJU Framwork
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "board.h"
#include <stdio.h>

/* === Macros definitions ====================================================================== */

/* === Private data type declarations ========================================================== */

typedef struct {
    uint32_t port;
    uint32_t pin;
} led_t;

typedef struct {
    led_t red;
    led_t green;
    led_t blue;
} led_rgb_t;

/* === Private variable declarations =========================================================== */

static const led_rgb_t LED_RGB = {
    .green =
        {
            .port = GPIOA,
            .pin = GPIO_PIN_1,
        },
    .blue =
        {
            .port = GPIOA,
            .pin = GPIO_PIN_2,
        },
    .red =
        {
            .port = GPIOC,
            .pin = GPIO_PIN_13,
        },
};

/* === Private function declarations =========================================================== */

#if defined(CORTEX_M)
void SysTick_Handler(void);

volatile uint64_t get_timer_value(void);
#endif

void delay_1ms(uint32_t count);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

#if defined(CORTEX_M)
static uint64_t timer_value;
#endif

/* === Private function implementation ========================================================= */

#if defined(CORTEX_M)
void SysTick_Handler(void) {
    timer_value++;
}

volatile uint64_t get_timer_value(void) {
    return timer_value * SystemCoreClock / 1000U / 4;
}
#endif

void delay_1ms(uint32_t count) {
    volatile uint64_t start_mtime, delta_mtime;

    volatile uint64_t tmp = get_timer_value();
    do {
        start_mtime = get_timer_value();
    } while (start_mtime == tmp);

    uint64_t delay_ticks = SystemCoreClock / 4; // 1 second
    delay_ticks = delay_ticks * count / 1000;

    do {
        delta_mtime = get_timer_value() - start_mtime;
    } while (delta_mtime < delay_ticks);
}

/* === Public function implementation ========================================================== */

int main(void) {

    BoardSetup();

#if defined(CORTEX_M)
    SysTick_Config(SystemCoreClock / 1000U);
#endif

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_init(LED_RGB.red.port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_RGB.red.pin);
    gpio_bit_set(LED_RGB.red.port, LED_RGB.red.pin);

    gpio_init(LED_RGB.green.port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_RGB.green.pin);
    gpio_bit_set(LED_RGB.green.port, LED_RGB.green.pin);

    gpio_init(LED_RGB.blue.port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_RGB.blue.pin);
    gpio_bit_set(LED_RGB.blue.port, LED_RGB.blue.pin);

    while (1) {
        gpio_bit_reset(LED_RGB.red.port, LED_RGB.red.pin);
        delay_1ms(1000);

        gpio_bit_set(LED_RGB.red.port, LED_RGB.red.pin);
        delay_1ms(1000);

        gpio_bit_reset(LED_RGB.green.port, LED_RGB.green.pin);
        delay_1ms(1000);

        gpio_bit_set(LED_RGB.green.port, LED_RGB.green.pin);
        delay_1ms(1000);

        gpio_bit_reset(LED_RGB.blue.port, LED_RGB.blue.pin);
        delay_1ms(1000);

        gpio_bit_set(LED_RGB.blue.port, LED_RGB.blue.pin);
        delay_1ms(1000);
    }
    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
