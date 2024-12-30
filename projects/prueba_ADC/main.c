/************************************************************************************************
Copyright ...

SPDX-License-Identifier: MIT
*************************************************************************************************/

/** \brief PWM control with ADC input
 **
 ** \addtogroup samples Samples
 ** \brief Samples applications with MUJU Framework
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "board.h"
#include <stdio.h>

/* === Macros definitions ====================================================================== */

#define PERIODO 20000
#define DELTA   (PERIODO / 20)

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

void delay_1ms(uint32_t count);

void gpio_config(void);

void timer_config(void);

void adc_config(void);

uint16_t adc_read(void);

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

/**
    \brief      Configure the GPIO ports PB8 and PB0
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_config(void) {
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /* Configure PB8 (TIMER3 CH2) as alternate function */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    /* Configure PB0 (ADC01_IN8) as analog input */
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
}

/**
    \brief      Configure the TIMER3 peripheral for PWM
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(void) {
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = 107;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = PERIODO;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER3, TIMER_CH_2, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, 0);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_2, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(TIMER3);
    timer_enable(TIMER3);
}

/**
    \brief      Configure the ADC to read from PB0 (ADC01_IN8)
    \param[in]  none
    \param[out] none
    \retval     none
  */
void adc_config(void) {
    rcu_periph_clock_enable(RCU_ADC1);

    adc_deinit(ADC1);
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);

    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);

    adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_9, ADC_SAMPLETIME_55POINT5);
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);

    adc_enable(ADC1);
    delay_1ms(1); // Small delay for ADC stabilization
    adc_calibration_enable(ADC1);
}

/**
    \brief      Read ADC value from PB0
    \param[in]  none
    \param[out] none
    \retval     12-bit ADC value (0-4095)
  */
uint16_t adc_read(void) {
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);
    while (!adc_flag_get(ADC1, ADC_FLAG_EOC));
    return adc_regular_data_read(ADC1);
}

/* === Public function implementation ========================================================== */

int main(void) {
    uint16_t adc_value;
    uint32_t pwm_value;

    BoardSetup();

    gpio_config();
    timer_config();
    adc_config();

    while (1) {
        // Read ADC value
        adc_value = adc_read();

        // Scale ADC value to PWM range (0 to PERIODO)
        pwm_value = (adc_value * PERIODO) / 4095;

        // Update PWM duty cycle
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, pwm_value);

        delay_1ms(10); // Small delay for smooth operation
    }

    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */