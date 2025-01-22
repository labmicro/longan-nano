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
#include <time.h>

/* === Macros definitions ====================================================================== */

//! Puerto I2C a utilizar para la comunicación con el RTC
#define I2C_PUERTO       I2C0

//! Señal para habilidar el reloj del puerto I2C
#define I2C_PUERTO_CLOCK RCU_I2C0

//! Puerto GPIO por el que salen las señales del puerto I2C
#define I2C_GPIO         GPIOB

//! Señal para habilidar el reloj del puerto GPIO usado por el I2C
#define I2C_GPIO_CLOCK   RCU_GPIOB

//! Numero del terminal que se utiliza para la señal SDA del puerto I2C
#define I2C_GPIO_SDA     GPIO_PIN_7

//! Numero del terminal que se utiliza para la señal SCL del puerto I2C
#define I2C_GPIO_SCL     GPIO_PIN_6

//! Frecuencia a la que opera el puerto I2C
#define I2C_FRECUENCIA   100000

//! Dirección del integrado RTC en el bus I2C
#define I2C_DIRECCION    0x6F

//! Dirección del registro de los segundos actuales en el RTC
#define RTCSEC           0x00

//! Dirección del registro de los minutos actuales en el RTC
#define RTCMIN           0x01

//! Dirección del registro de la hora actual en el RTC
#define RTCHOUR          0x02

//! Dirección del registro del dia actual de la semana en el RTC
#define RTCWKDAY         0x03

//! Dirección del registro del dia actual del mes en el RTC
#define RTCDATE          0x04

//! Dirección del registro del mes actual en el RTC
#define RTCMTH           0x05

//! Dirección del registro del año actual en el RTC
#define RTCYEAR          0x06

//! Bit que informa el estado de funciomiento del RTC
#define ST               (1 << 7)

//! Bit que habilita el oscilador del RTC
#define OSCRUN           (1 << 5)

//! Bit que indica un corte de energia en el RTC
#define PWRFAIL          (1 << 4)

//! Bit que habilita el uso de la bateria de respaldo en el RTC
#define VBATEN           (1 << 3)

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

void delay_1ms(uint32_t count);

static uint8_t bin_to_bcd(uint8_t valor);

static uint8_t bcd_to_bin(uint8_t valor);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* 2025-01-22 12:40:45 */
static const struct tm * inital_date = &(const struct tm){
    .tm_sec = 45,   //! Segundos de la hora actual, desde 0 a 59
    .tm_min = 40,   //! Minutos de la hora actual, desde 0 a 59
    .tm_hour = 12,  //! Hora actual, desde 0 hasta 23
    .tm_mday = 22,  //! Dia del mes actual, desde 1 hasta 31
    .tm_mon = 0,    //! Mes actual, desde 0 (Enero) hasta 11 (Diciembre)
    .tm_year = 125, //! Año actual menos 1900
    .tm_wday = 3,   //! Dia de la semana, desde 0 (Domingo) hasta el 6 (Sábado))
};

/* === Private function implementation ========================================================= */

static uint8_t bcd_to_bin(uint8_t valor) {
    return ((10 * ((valor & 0xF0) >> 4)) + (valor & 0x0F));
}

static uint8_t bin_to_bcd(uint8_t valor) {
    return (((valor / 10) % 10) << 4) | (valor % 10);
}

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
    rcu_periph_clock_enable(I2C_GPIO_CLOCK);

    /* enable I2C clock */
    rcu_periph_clock_enable(I2C_PUERTO_CLOCK);

    /* connect pin to I2C_PUERTO_SCL in mode open-drain */
    gpio_init(I2C_GPIO, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C_GPIO_SCL);
    /* connect pin to I2C_PUERTO_SDA in mode open-drain */
    gpio_init(I2C_GPIO, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C_GPIO_SDA);

    i2c_deinit(I2C_PUERTO);
    i2c_clock_config(I2C_PUERTO, I2C_FRECUENCIA, I2C_DTCY_2);
    i2c_enable(I2C_PUERTO);
}

void i2c_start_transaction(uint32_t i2c_periph) {
    /* wait until I2C bus is idle */
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY));
}

void i2c_write_data(uint32_t i2c_periph, uint8_t device, uint8_t data[], ssize_t size) {
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, device << 1, I2C_TRANSMITTER);

    i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    for (int count = 0; count < size; count++) {
        /* wait until the transmit data buffer is empty */
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));
        /* data transmission with the number of first register to read */
        i2c_data_transmit(i2c_periph, data[count]);

        if (size - 1 == count) {
            /* wait until the second last data byte is received into the shift register */
            while (!i2c_flag_get(i2c_periph, I2C_FLAG_BTC));
            /* disable acknowledge */
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        }
    }
}

void i2c_read_data(uint32_t i2c_periph, uint8_t device, uint8_t data[], ssize_t size) {
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, device << 1, I2C_RECEIVER);

    /* if data size are below two bytes disable ACK before clear ADDSEND bit */
    if (size < 2) {
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
    } else {
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
    }

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    for (int count = 0; count < size; count++) {
        if (size - 3 == count) {
            /* wait until the second last data byte is received into the shift register */
            while (!i2c_flag_get(i2c_periph, I2C_FLAG_BTC));
            /* disable acknowledge */
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        }
        /* wait until the RBNE bit is set */
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE));
        /* read a data from I2C_DATA */
        data[count] = i2c_data_receive(i2c_periph);
    }
}

void i2c_select_register(uint32_t i2c_periph, uint8_t device, uint8_t address) {
    i2c_write_data(i2c_periph, device, &address, 1);
}

uint8_t i2c_read_byte(uint32_t i2c_periph, uint8_t device) {
    i2c_start_on_bus(i2c_periph);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, device << 1, I2C_RECEIVER);

    /* disable ACK before clearing ADDSEND bit */
    i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));

    /* clear ADDSEND bit */
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    /* wait until the RBNE bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE));

    /* read a data from I2C_DATA */
    return i2c_data_receive(i2c_periph);
}

void i2c_terminate_transaction(uint32_t i2c_periph) {
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c_periph);
}

void rtc_update(const struct tm * datetime) {
    uint8_t data[8];

    data[0] = RTCSEC;
    data[RTCSEC + 1] = bin_to_bcd(datetime->tm_sec) | ST;
    data[RTCMIN + 1] = bin_to_bcd(datetime->tm_min);
    data[RTCHOUR + 1] = bin_to_bcd(datetime->tm_hour);
    data[RTCWKDAY + 1] = bin_to_bcd(datetime->tm_wday + 1) | OSCRUN | VBATEN;
    data[RTCDATE + 1] = bin_to_bcd(datetime->tm_mday);
    data[RTCMTH + 1] = bin_to_bcd(datetime->tm_mon + 1);
    data[RTCYEAR + 1] = bin_to_bcd(datetime->tm_year - 100);

    i2c_start_transaction(I2C_PUERTO);
    i2c_write_data(I2C_PUERTO, I2C_DIRECCION, data, sizeof(data));
    i2c_terminate_transaction(I2C_PUERTO);
}

void rtc_query(struct tm * datetime) {
    uint8_t data[7];

    i2c_start_transaction(I2C_PUERTO);
    i2c_select_register(I2C_PUERTO, I2C_DIRECCION, RTCSEC);
    i2c_read_data(I2C_PUERTO, I2C_DIRECCION, data, sizeof(data));
    i2c_terminate_transaction(I2C_PUERTO);

    datetime->tm_sec = bcd_to_bin(data[RTCSEC] & 0x7F);
    datetime->tm_min = bcd_to_bin(data[RTCMIN]);
    datetime->tm_hour = bcd_to_bin(data[RTCHOUR] & 0x3F);
    datetime->tm_wday = bcd_to_bin(data[RTCWKDAY] & 0x07) - 1;
    datetime->tm_mday = bcd_to_bin(data[RTCDATE] & 0x3F);
    datetime->tm_mon = bcd_to_bin(data[RTCMTH] & 0x1F) - 1;
    datetime->tm_year = bcd_to_bin(data[RTCYEAR]) + 100;
};

void format_datetime(const struct tm * datetime, char * data, ssize_t size) {
    snprintf(data, size, "%04d-%02d-%02d %02d:%02d:%02d", datetime->tm_year + 1900,
             datetime->tm_mon + 1, datetime->tm_mday, datetime->tm_hour, datetime->tm_min,
             datetime->tm_sec);
}

/* === Public function implementation ========================================================== */

int main() {
    char cadena[20];
    struct tm actual;

    BoardSetup();
    ConsoleSetup();

    i2c_init();

    printf("Configurando el RTC...\n");
    rtc_update(inital_date);
    printf("done\n\n");

    for (;;) {
        printf("Reading RTC \n");
        rtc_query(&actual);

        format_datetime(&actual, cadena, sizeof(cadena));
        printf("%s \n", cadena);

        printf("done\n\n");
        delay_1ms(30 * 1000);
    }

    return 0;
}

/* retarget the C library printf function to the USART */
int _put_char(int ch) {
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
