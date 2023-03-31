/*!
    \file    main.h
    \brief   the header file of main 

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x 
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

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

#ifndef MAIN_H
#define MAIN_H
#include "gd32f30x_gpio.h"

#define __PACKED		__attribute__ ((__packed__))
/* 角色 */
#define ROLE (e_master)

/* 通信速率：1MBps */
#define CAN_BAUDRATE 1000
/* 500kBps */
/* #define CAN_BAUDRATE  500 */
/* 250kBps */
/* #define CAN_BAUDRATE  250 */
/* 125kBps */
/* #define CAN_BAUDRATE  125 */
/* 100kBps */
/* #define CAN_BAUDRATE  100 */
/* 50kBps */
/* #define CAN_BAUDRATE  50 */
/* 20kBps */
/* #define CAN_BAUDRATE  20 */

/* led spark function */
/* void led_spark(void); */
void __can_init__();
void can_send_data(const uint8_t send_data, const uint32_t tx_sfid);
void receive_data();
void can_config();
void nvic_config(void);
void led_on(LED_e led);
void led_off(LED_e led);
void led_init(void);
void can_gpio_config(void);
void uart0_init();

typedef enum
{
    e_slave_1 = 0x01,
    e_slave_2 = 0x02,
    e_slave_3 = 0x03,
    e_slave_4 = 0x04,
    e_slave_5 = 0x05,
    e_slave_6 = 0x06,
    e_slave_7 = 0x07,
    e_slave_8 = 0x08,
    e_slave_9 = 0x09,
    e_slave_10 = 0x0A,
    e_slave_11 = 0x0B,
    e_slave_12 = 0x0C,
    e_slave_13 = 0x0D,
    e_slave_14 = 0x0E,
    e_slave_15 = 0x0F,
    e_slave_16 = 0x10,
    e_slave_17 = 0x11,
    e_slave_18 = 0x12,
    e_slave_19 = 0x13,
    e_slave_20 = 0x14,
    e_slave_21 = 0x15,
    e_slave_22 = 0x16,
    e_slave_23 = 0x17,
    e_slave_24 = 0x18,
    e_slave_25 = 0x19,
    e_slave_26 = 0x1A,
    e_slave_27 = 0x1B,
    e_slave_28 = 0x1C,
    e_slave_29 = 0x1D,
    e_slave_30 = 0x1E,
    e_slave_31 = 0x1F,
    e_slave_32 = 0x20,
    e_master = 0x51
} ROLE_e;

typedef enum
{
    e_green_led,
    e_red_led
} LED_e;

#endif /* MAIN_H */

