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

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

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

    #if defined(CORTEX_M)
    SysTick_Config(SystemCoreClock / 1000U);
    #endif

    /* Habilitar el reloj para el puerto GPIOB */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* Configurar el pin PA3 como salida push-pull */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

    /* Asegurarse de que el LED esté apagado inicialmente */
    gpio_bit_reset(GPIOB, GPIO_PIN_13);
    gpio_bit_reset(GPIOB, GPIO_PIN_14);
    gpio_bit_reset(GPIOB, GPIO_PIN_15);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);

    while (1) {
        gpio_bit_set(GPIOB, GPIO_PIN_13);
        delay_1ms(100); // Esperar 1 segundo

        gpio_bit_reset(GPIOB, GPIO_PIN_13);
        delay_1ms(100); // Esperar 1 segundo
        
        gpio_bit_set(GPIOB, GPIO_PIN_14);
        delay_1ms(100); // Esperar 1 segundo

        
        gpio_bit_reset(GPIOB, GPIO_PIN_14);
        delay_1ms(100); // Esperar 1 segundo


        
        gpio_bit_set(GPIOB, GPIO_PIN_15);
        delay_1ms(100); // Esperar 1 segundo

        
        gpio_bit_reset(GPIOB, GPIO_PIN_15);
        delay_1ms(100); // Esperar 1 segundo
        

        
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(100); // Esperar 1 segundo

        
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(100); // Esperar 1 segundo
    }
}
