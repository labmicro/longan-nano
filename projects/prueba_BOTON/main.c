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

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

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
    /* Habilitar el reloj para el puerto GPIOA */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* Configurar PA4 como salida push-pull */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    /* Configurar PA7 como entrada flotante */
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_15);

    /* Asegurarse de que el LED esté apagado inicialmente */
    gpio_bit_reset(GPIOA, GPIO_PIN_4);

    while (1) {
        /* Leer el estado del pulsador en PA7 */
        if (gpio_input_bit_get(GPIOC, GPIO_PIN_15)) {
            /* Encender el LED (PA4 en alto) si el pulsador está presionado */
            gpio_bit_set(GPIOA, GPIO_PIN_4);
        } else {
            /* Apagar el LED (PA4 en bajo) si el pulsador no está presionado */
            gpio_bit_reset(GPIOA, GPIO_PIN_4);
        }
    }
}
