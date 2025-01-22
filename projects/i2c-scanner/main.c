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
#include <string.h>

/* === Macros definitions ====================================================================== */

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

void delay_1ms(uint32_t count);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

void ConsoleSetup(void) {
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

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

void i2c_init(void) {
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* enable I2C clock */
    rcu_periph_clock_enable(RCU_I2C1);

    /* connect PB10 to I2C1_SCL in mode open-drain */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    /* connect PB11 to I2C1_SDA in mode open-drain */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    /* connect PB6 to I2C0_SCL in mode open-drain */
    // gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    /* connect PB7 to I2C0_SDA in mode open-drain */
    // gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    i2c_deinit(I2C1);
    i2c_clock_config(I2C1, 100000, I2C_DTCY_2);
    i2c_enable(I2C1);
}

static void probe_address(uint32_t i) {
    printf("0x%lx, ", i);

    while (i2c_flag_get(I2C1, I2C_FLAG_I2CBSY))
        ;
    i2c_start_on_bus(I2C1);
    while (!i2c_flag_get(I2C1, I2C_FLAG_SBSEND))
        ;
    // it's cleared by getting the flag, ie. reading I2C_STAT0
    i2c_master_addressing(I2C1, i << 1, I2C_TRANSMITTER);
    uint32_t k = 0;
    while (!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND)) {
        if (i2c_flag_get(I2C1, I2C_FLAG_AERR)) {
            i2c_flag_clear(I2C1, I2C_FLAG_AERR);
            i2c_stop_on_bus(I2C1);
            return;
        }
        // in case no bus is connected
        if (k++ > 1000 * 1000) {
            i2c_stop_on_bus(I2C1);
            return;
        }
    }

    printf("Found device on address: 0x%lx (%lu)\n", i, i);
    // NB: it's cleared by reading I2C_STAT0 _and_ I2C_STAT1
    i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);

    i2c_stop_on_bus(I2C1);

    // wait for stop being sent
    while (I2C_CTL0(I2C1) & I2C_CTL0_STOP)
        ;
}

static void scan() {
    for (uint32_t i = 8; i < 128; ++i) probe_address(i);
}

/* === Public function implementation ========================================================== */

int main() {
    BoardSetup();
    ConsoleSetup();

    i2c_init();

    for (;;) {
        printf("Scanning I2C bus at 100 kHz ...\n");
        scan();

        printf("Scanning I2C bus at 400 kHz ...\n");
        i2c_clock_config(I2C1, 400000, I2C_DTCY_2);
        scan();

        printf("done\n\n");

        i2c_clock_config(I2C1, 100000, I2C_DTCY_2);
        delay_1ms(30 * 1000);
    }

    return 0;
}

/* retarget the C library printf function to the USART */
int _put_char(int ch) {
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
        ;
    return ch;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
