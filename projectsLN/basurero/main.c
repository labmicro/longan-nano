/*!
    \file  main.c
    \brief master receiver

    \version 2019-6-5, V1.0.0, firmware for GD32VF103

*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include "gd32vf103v_eval.h"
#include <stdio.h>
#include "board.h"

#define EVAL_COM0                        USART0

#define I2C_10BIT_ADDRESS    0

#define I2C0_OWN_ADDRESS7    0x72
#define I2C0_SLAVE_ADDRESS7  0x82
#define I2C0_SLAVE_ADDRESS10 0x0322

uint8_t i2c_receiver[16];

void rcu_config(void);
void gpio_config(void);
void i2c_config(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {
    uint8_t i;
    uint8_t slave10_first_byte, slave10_second_byte;

    /* configure USART */
    gd_eval_com_init(EVAL_COM0);

    printf("\r\n I2C Start\n");

    /* RCU config */
    rcu_config();
    /* GPIO config */
    gpio_config();
    /* I2C config */
    i2c_config();

uint8_t rtc_data[7];

/* Iniciar la comunicación I2C */
i2c_start_on_bus(I2C0);
while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));

/* Enviar dirección del esclavo en modo transmisión */
i2c_master_addressing(I2C0, 0xD0, I2C_TRANSMITTER); // DS3231 dirección escritura
while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

/* Enviar dirección del registro de segundos (0x00) */
i2c_data_transmit(I2C0, 0x00);
while (!i2c_flag_get(I2C0, I2C_FLAG_TBE));

/* Repetir START para cambiar a modo lectura */
i2c_start_on_bus(I2C0);
while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));

/* Enviar dirección del esclavo en modo recepción */
i2c_master_addressing(I2C0, 0xD1, I2C_RECEIVER); // DS3231 dirección lectura
while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

/* Recibir los 7 bytes de la fecha/hora */
for (i = 0; i < 7; i++) {
    while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    rtc_data[i] = i2c_data_receive(I2C0);
}

/* Enviar STOP */
i2c_stop_on_bus(I2C0);
while (I2C_CTL0(I2C0) & 0x0200);

/* Convertir BCD a decimal */
uint8_t segundos = ((rtc_data[0] >> 4) * 10) + (rtc_data[0] & 0x0F);
uint8_t minutos  = ((rtc_data[1] >> 4) * 10) + (rtc_data[1] & 0x0F);
uint8_t horas    = ((rtc_data[2] >> 4) * 10) + (rtc_data[2] & 0x0F);

/* Imprimir la hora */
printf("Hora: %02d:%02d:%02d\n", horas, minutos, segundos);

#if I2C_10BIT_ADDRESS
    slave10_first_byte = (0xF0) | (uint8_t)((I2C0_SLAVE_ADDRESS10 & 0x0300) >> 7);
    /* send slave address first byte to I2C bus */
    i2c_master_addressing(I2C0, slave10_first_byte, I2C_TRANSMITTER);
    /* wait until ADD10SEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADD10SEND))
        ;
    /* the second byte contains the remaining 8 bits of the 10-bit address */
    slave10_second_byte = (uint8_t)(I2C0_SLAVE_ADDRESS10 & 0x00FF);
    /* send slave address 2nd byte to I2C bus */
    i2c_master_addressing(I2C0, slave10_second_byte, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
        ;
    /* send slave address first byte to I2C bus */
    i2c_master_addressing(I2C0, slave10_first_byte, I2C_RECEIVER);
#else
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_RECEIVER);
#endif
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

    printf("\r\n I2C SUCCESS\n");

    while (1) {
    }
}

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void) {
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      cofigure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void) {
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

/*!
    \brief      cofigure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void) {
    /* I2C clock configure */
    i2c_clock_config(I2C0, 400000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

/* retarget the C library printf function to the usart */
int fputc(int ch, FILE * f) {
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE))
        ;
    return ch;
}
