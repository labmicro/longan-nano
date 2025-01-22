#include "gd32vf103.h"
#include <stdio.h>

/* Parámetros */
#define PERIODO 20000      // Período del PWM
#define MAX_ENCODER_COUNT 100  // Máxima posición del encoder
#define MIN_ENCODER_COUNT 0    // Mínima posición del encoder

/* Declaración de funciones */
void gpio_config(void);
void timer_pwm_config(void);
void timer_encoder_config(void);
void delay_1ms(uint32_t count);

/* Configuración de GPIO */
void gpio_config(void) {
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);  // PWM en PB8
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4 | GPIO_PIN_5);  // Entradas del encoder PB4 y PB5
}

/* Configuración de temporizador para PWM */
void timer_pwm_config(void) {
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);
    timer_struct_para_init(&timer_initpara);

    timer_initpara.prescaler = 10;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = PERIODO;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_channel_output_config(TIMER3, TIMER_CH_2, &timer_ocinitpara);

    timer_channel_output_mode_config(TIMER3, TIMER_CH_2, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(TIMER3);
    timer_enable(TIMER3);
}

/* Configuración de temporizador para el encoder */
void timer_encoder_config(void) {
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);
    timer_deinit(TIMER2);

    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = 0;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = MAX_ENCODER_COUNT;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    timer_quadrature_decoder_mode_config(TIMER2, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING);
    timer_autoreload_value_config(TIMER2, MAX_ENCODER_COUNT);
    timer_enable(TIMER2);
}

/* Retardo en milisegundos */
void delay_1ms(uint32_t count) {
    uint64_t start_mtime, delta_mtime;
    start_mtime = get_timer_value();
    uint64_t delay_ticks = SystemCoreClock / 4000 * count;
    do {
        delta_mtime = get_timer_value() - start_mtime;
    } while (delta_mtime < delay_ticks);
}

int main(void) {
    int encoder_value = 0;

    gpio_config();
    timer_pwm_config();
    timer_encoder_config();

    while (1) {
        /* Leer posición del encoder */
        encoder_value = timer_counter_read(TIMER2) * 5;

        /* Limitar el valor dentro del rango permitido */
        if (encoder_value > MAX_ENCODER_COUNT) {
            encoder_value = MAX_ENCODER_COUNT;
        } else if (encoder_value < MIN_ENCODER_COUNT) {
            encoder_value = MIN_ENCODER_COUNT;
        }

        /* Ajustar ciclo de trabajo del PWM */
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, (PERIODO * encoder_value) / MAX_ENCODER_COUNT);

        delay_1ms(100);
    }

    return 0;
}
