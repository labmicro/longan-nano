/*!
    \file  usbh_hid_keybd.c
    \brief this file is the application layer for usb host hid keyboard handling
           qwerty and azerty keyboard are supported as per the selection in
           usbh_hid_keybd.h

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

#include "usbh_hid_keybd.h"
#include <stdio.h>

static void keybrd_init(void);
static void keybrd_decode(uint8_t * pbuf);

hid_proc HID_KEYBRD_cb = {keybrd_init, keybrd_decode};

/* LOCAL CONSTANTS */
static const uint8_t HID_KEYBRD_Codes[] = {
    0,   0,   0,   0,   31,  50,  48,  33,
    19,  34,  35,  36,  24,  37,  38,  39, /* 0x00 - 0x0F */
    52,  51,  25,  26,  17,  20,  32,  21,
    23,  49,  18,  47,  22,  46,  2,   3, /* 0x10 - 0x1F */
    4,   5,   6,   7,   8,   9,   10,  11,
    43,  110, 15,  16,  61,  12,  13,  27, /* 0x20 - 0x2F */
    28,  29,  42,  40,  41,  1,   53,  54,
    55,  30,  112, 113, 114, 115, 116, 117, /* 0x30 - 0x3F */
    118, 119, 120, 121, 122, 123, 124, 125,
    126, 75,  80,  85,  76,  81,  86,  89, /* 0x40 - 0x4F */
    79,  84,  83,  90,  95,  100, 105, 106,
    108, 93,  98,  103, 92,  97,  102, 91, /* 0x50 - 0x5F */
    96,  101, 99,  104, 45,  129, 0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0x60 - 0x6F */
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0x70 - 0x7F */
    0,   0,   0,   0,   0,   107, 0,   56,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0x80 - 0x8F */
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0x90 - 0x9F */
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0xA0 - 0xAF */
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0xB0 - 0xBF */
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, /* 0xC0 - 0xCF */
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,  /* 0xD0 - 0xDF */
    58,  44,  60,  127, 64,  57,  62,  128 /* 0xE0 - 0xE7 */
};

#ifdef QWERTY_KEYBOARD
static const int8_t HID_KEYBRD_Key[] = {
    '\0', '`',  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  '0',  '-',  '=',  '\0',
    '\r', '\t', 'q',  'w',  'e',  'r',  't',  'y',  'u',  'i',  'o',  'p',  '[',  ']',  '\\',
    '\0', 'a',  's',  'd',  'f',  'g',  'h',  'j',  'k',  'l',  ';',  '\'', '\0', '\n', '\0',
    '\0', 'z',  'x',  'c',  'v',  'b',  'n',  'm',  ',',  '.',  '/',  '\0', '\0', '\0', '\0',
    '\0', ' ',  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\r', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '7',  '4',  '1',  '\0', '/',  '8',  '5',  '2',  '0',  '*',  '9',  '6',  '3',  '.',
    '-',  '+',  '\0', '\n', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};

static const int8_t HID_KEYBRD_ShiftKey[] = {
    '\0', '~',  '!',  '@',  '#',  '$',  '%',  '^',  '&',  '*',  '(',  ')',  '_',  '+',  '\0',
    '\0', '\0', 'Q',  'W',  'E',  'R',  'T',  'Y',  'U',  'I',  'O',  'P',  '{',  '}',  '|',
    '\0', 'A',  'S',  'D',  'F',  'G',  'H',  'J',  'K',  'L',  ':',  '"',  '\0', '\n', '\0',
    '\0', 'Z',  'X',  'C',  'V',  'B',  'N',  'M',  '<',  '>',  '?',  '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};

#else

static const int8_t HID_KEYBRD_Key[] = {
    '\0', '`',  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  '0',  '-',  '=',  '\0',
    '\r', '\t', 'a',  'z',  'e',  'r',  't',  'y',  'u',  'i',  'o',  'p',  '[',  ']',  '\\',
    '\0', 'q',  's',  'd',  'f',  'g',  'h',  'j',  'k',  'l',  'm',  '\0', '\0', '\n', '\0',
    '\0', 'w',  'x',  'c',  'v',  'b',  'n',  ',',  ';',  ':',  '!',  '\0', '\0', '\0', '\0',
    '\0', ' ',  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\r', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '7',  '4',  '1',  '\0', '/',  '8',  '5',  '2',  '0',  '*',  '9',  '6',  '3',  '.',
    '-',  '+',  '\0', '\n', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};

static const int8_t HID_KEYBRD_ShiftKey[] = {
    '\0', '~',  '!',  '@',  '#',  '$',  '%',  '^',  '&',  '*',  '(',  ')',  '_',  '+',  '\0',
    '\0', '\0', 'A',  'Z',  'E',  'R',  'T',  'Y',  'U',  'I',  'O',  'P',  '{',  '}',  '*',
    '\0', 'Q',  'S',  'D',  'F',  'G',  'H',  'J',  'K',  'L',  'M',  '%',  '\0', '\n', '\0',
    '\0', 'W',  'X',  'C',  'V',  'B',  'N',  '?',  '.',  '/',  '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
#endif

/*!
    \brief      initialize the keyboard function.
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void keybrd_init(void) {
    /* call user init*/
    usr_keybrd_init();
}

/*!
    \brief      decode the pressed keys.
    \param[in]  pbuf: pointer to the HID IN report data buffer
    \param[out] none
    \retval     none
*/
static void keybrd_decode(uint8_t * pbuf) {
    static uint8_t shift;
    static uint8_t keys[KBR_MAX_NBR_PRESSED];
    static uint8_t keys_new[KBR_MAX_NBR_PRESSED];
    static uint8_t keys_last[KBR_MAX_NBR_PRESSED];
    static uint8_t key_newest;
    static uint8_t nbr_keys;
    static uint8_t nbr_keys_new;
    static uint8_t nbr_keys_last;
    uint8_t ix;
    uint8_t jx;
    uint8_t error;
    uint8_t output;

    nbr_keys = 0U;
    nbr_keys_new = 0U;
    nbr_keys_last = 0U;
    key_newest = 0x00U;

    /* check if shift key is pressed */
    if ((KBD_LEFT_SHIFT == pbuf[0]) || (KBD_RIGHT_SHIFT == pbuf[0])) {
        shift = TRUE;
    } else {
        shift = FALSE;
    }

    error = FALSE;

    /* check for the value of pressed key */
    for (ix = 2U; ix < 2U + KBR_MAX_NBR_PRESSED; ix++) {
        if ((0x01U == pbuf[ix]) || (0x02U == pbuf[ix]) || (0x03U == pbuf[ix])) {
            error = TRUE;
        }
    }

    if (TRUE == error) {
        return;
    }

    nbr_keys = 0U;
    nbr_keys_new = 0U;

    for (ix = 2U; ix < 2U + KBR_MAX_NBR_PRESSED; ix++) {
        if (0U != pbuf[ix]) {
            keys[nbr_keys] = pbuf[ix];
            nbr_keys++;

            for (jx = 0U; jx < nbr_keys_last; jx++) {
                if (pbuf[ix] == keys_last[jx]) {
                    break;
                }
            }

            if (jx == nbr_keys_last) {
                keys_new[nbr_keys_new] = pbuf[ix];
                nbr_keys_new++;
            }
        }
    }

    if (1U == nbr_keys_new) {
        key_newest = keys_new[0];

        if (TRUE == shift) {
            output = (uint8_t)HID_KEYBRD_ShiftKey[HID_KEYBRD_Codes[key_newest]];
        } else {
            output = (uint8_t)HID_KEYBRD_Key[HID_KEYBRD_Codes[key_newest]];
        }

        /* call user process handle */
        usr_keybrd_process_data(output);
    } else {
        key_newest = 0x00U;
    }

    nbr_keys_last = nbr_keys;

    for (ix = 0U; ix < KBR_MAX_NBR_PRESSED; ix++) {
        keys_last[ix] = keys[ix];
    }
}
