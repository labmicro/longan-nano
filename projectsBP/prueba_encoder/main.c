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
#include <stdbool.h>

/* === Macros definitions ====================================================================== */

#define PERIODO 20000
#define DELTA   (PERIODO / 20)

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

#if defined(CORTEX_M)
void SysTick_Handler(void);

volatile uint64_t get_timer_value(void);
#endif

void delay_1ms(uint32_t count);

void gpio_config(void);

void timer_config(void);

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

/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_config(void) {
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA1 PA2 PA3(TIMER1 CH1 CH2 CH3) as alternate function*/
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    // gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    // gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(void) {
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler = 10;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = PERIODO;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER3, TIMER_CH_2, &timer_ocinitpara);
    // timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocinitpara);
    // timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocinitpara);

    /* CH1 configuration in PWM mode1,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, 0);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_2, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(TIMER3);
    /* auto-reload preload enable */
    timer_enable(TIMER3);

    rcu_periph_clock_enable(RCU_TIMER0);

    timer_quadrature_decoder_mode_config(TIMER0, TIMER_QUAD_DECODER_MODE2, TIMER_IC_POLARITY_RISING,
                                         TIMER_IC_POLARITY_RISING);
    timer_autoreload_value_config(TIMER0, 0xFFFF);
    timer_enable(TIMER0);
}

/* === Public function implementation ========================================================== */

int main(void) {
    int value = 1000;
    int change = DELTA;
    bool flip;

    BoardSetup();
    
    #if defined(CORTEX_M)
        SysTick_Config(SystemCoreClock / 1000U);
    #endif


    gpio_config();
    timer_config();

    while (1) {
        value = timer_counter_read(TIMER0) * 1000;

        flip = false;
        if (value > PERIODO) {
            value = PERIODO;
            change = -DELTA;
            flip = true;
        } else if (value < 0) {
            value = 0;
            change = DELTA;
            flip = true;
        }

        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, value);

        if (flip) {
            delay_1ms(1000);
        } else {
            delay_1ms(100);
        }
    }

    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
