/*!
    \file    main.c
    \brief   communication_among_CANS in normal mode
    
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

#include "gd32f30x.h"
#include <stdio.h>
#include "gd32f307c_eval.h"

/* select CAN baudrate */
/* 1MBps */
#define CAN_BAUDRATE  1000
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

FlagStatus can0_receive_flag;
FlagStatus can1_receive_flag;
FlagStatus can0_error_flag;
FlagStatus can1_error_flag;

can_trasnmit_message_struct transmit_message;
can_receive_message_struct receive_message;

void nvic_config(void);
void led_config(void);
void can_gpio_config(void);
void can_config(void);
/* 这是一段嵌入式 C 语言代码，用于在 GD32F30x 双 CAN 总线上进行通信测试。
该代码包含了对 GPIO、USART、按键、LED 等的配置和初始化，以及 CAN 总线的配置、消息发送和接收。
代码中使用了循环来检测按键的状态并发送相应的数据。
当接收到 CAN 总线上的数据时，代码会将其打印出来并控制 LED 灯的状态。
此外，该代码还包含了一些错误处理机制，例如发送超时等情况 */

int main(void)
{
    uint8_t i = 0;                // 定义一个8位无符号整数i并初始化为0用于循环计数
    uint32_t timeout = 0xFFFF;    // 定义一个32位无符号整数timeout并初始化为0xFFFF超时时间，用于等待CAN模块初始化完成
    uint8_t transmit_mailbox = 0; // 定义一个8位无符号整数transmit_mailbox并初始化为0，发送邮箱编号，用于标识发送的CAN消息在哪个邮箱中

    can0_receive_flag = RESET; // 将CAN0接收标志清零
    can1_receive_flag = RESET; // 将CAN1接收标志清零
    can0_error_flag = RESET;   // 将CAN0错误标志清零
    can1_error_flag = RESET;   // 将CAN1错误标志清零

    // 配置GPIO引脚
    can_gpio_config();

    // 配置中断向量表
    nvic_config();

    /* 配置USART串口通信 */
    gd_eval_com_init(EVAL_COM0);

    /* 配置Wakeup键和Tamper键 */
    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_GPIO);
    gd_eval_key_init(KEY_TAMPER, KEY_MODE_GPIO);

    printf("\r\nGD32F30x dual CAN test, please press Wakeup key or Tamper key to start communication!\r\n");
    /* 配置LED灯 */
    led_config();
    gd_eval_led_off(LED2); // 关闭LED2
    gd_eval_led_off(LED3); // 关闭LED3
    gd_eval_led_off(LED4); // 关闭LED4
    gd_eval_led_off(LED5); // 关闭LED5

    /* 初始化CAN和过滤器 */
    can_config();
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INT_RFNE0); // 使能CAN0的接收FIFO0非空中断
    can_interrupt_enable(CAN1, CAN_INT_RFNE0); // 使能CAN1的接收FIFO0非空中断

    /* initialize transmit message */
    transmit_message.tx_sfid = 0x7ab;     // 初始化发送消息的标准帧ID
    transmit_message.tx_efid = 0x00;      // 初始化扩展帧ID
    transmit_message.tx_ft = CAN_FT_DATA; // 初始化发送数据帧类型
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8; // 发送数据长度8字节
    /* 数据内容 */
    transmit_message.tx_data[0] = 0x00;
    transmit_message.tx_data[1] = 0xA1;
    transmit_message.tx_data[2] = 0xA2;
    transmit_message.tx_data[3] = 0xA3;
    transmit_message.tx_data[4] = 0xA4;
    transmit_message.tx_data[5] = 0xA5;
    transmit_message.tx_data[6] = 0xA6;
    transmit_message.tx_data[7] = 0xA7;

    while (1)
    {
        /* 检测是否按下Tamper按键 */
        if (0 == gd_eval_key_state_get(KEY_TAMPER)) // 如果按下Tamper按键
        {
            transmit_message.tx_data[0] = 0x55;            // 设置发送数据第一个字节为0x55
            transmit_message.tx_data[1] = 0xAA;            // 设置发送数据第二个字节为0xAA
            printf("\r\n can0 transmit data:");            // 打印提示信息
            for (i = 0; i < transmit_message.tx_dlen; i++) // 循环打印发送数据的每一个字节
            {
                printf(" %02x", transmit_message.tx_data[i]);
            }

            /* 发送消息 */
            transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
            /* 等待发送完成 */
            timeout = 0xFFFF;
            while ((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout))
            {
                timeout--;
            }
            /* 等待松开Tamper按键 */
            while (0 == gd_eval_key_state_get(KEY_TAMPER))
                ;
        }
        /* 检测是否按下Wakeup按键 */
        if (0 == gd_eval_key_state_get(KEY_WAKEUP)) // 如果按下Wakeup按键
        {
            transmit_message.tx_data[0] = 0xAA;            // 设置发送数据第一个字节为0xAA
            transmit_message.tx_data[1] = 0x55;            // 设置发送数据第二个字节为0x55
            printf("\r\n can1 transmit data:");            // 打印提示信息
            for (i = 0; i < transmit_message.tx_dlen; i++) // 循环打印发送数据的每一个字节
            {
                printf(" %02x", transmit_message.tx_data[i]);
            }
            /* 发送消息 */
            transmit_mailbox = can_message_transmit(CAN1, &transmit_message);
            /* 等待发送完成 */
            timeout = 0xFFFF;
            while ((CAN_TRANSMIT_OK != can_transmit_states(CAN1, transmit_mailbox)) && (0 != timeout))
            {
                timeout--;
            }
            /* 等待松开Wakeup按键 */
            while (0 == gd_eval_key_state_get(KEY_WAKEUP))
                ;
        }
        /* 如果CAN0正确接收到数据，打印接收到的数据 */
        if (SET == can0_receive_flag)
        {
            can0_receive_flag = RESET;
            printf("\r\n can0 receive data:");
            for (i = 0; i < receive_message.rx_dlen; i++)
            {
                printf(" %02x", receive_message.rx_data[i]);
            }
            gd_eval_led_toggle(LED4); // 交替切换LED4的状态
        }
        /* 如果CAN1正确接收到数据，打印接收到的数据 */
        if (SET == can1_receive_flag)
        {
            can1_receive_flag = RESET;         // 清除接收标志位
            gd_eval_led_toggle(LED5);          // 控制LED5灯闪烁
            printf("\r\n can1 receive data:"); // 打印CAN1接收数据的提示信息
            for (i = 0; i < receive_message.rx_dlen; i++)
            {
                printf(" %02x", receive_message.rx_data[i]); // 逐个字节打印CAN1接收到的数据
            }
        }
        /* 如果CAN0通信发生错误，则打印错误信息 */
        if (SET == can0_error_flag)
        {
            can0_error_flag = RESET;                 // 清除CAN0错误标志位
            printf("\r\n can0 communication error"); // 打印CAN0通信错误信息
        }
        /* 如果CAN1通信发生错误，则打印错误信息 */
        if (SET == can1_error_flag)
        {
            can1_error_flag = RESET;                 // 清除CAN1错误标志位
            printf("\r\n can1 communication error"); // 打印CAN1通信错误信息
        }
    }
}

/*
 * 函数名：can_config
 * 功能描述：配置CAN总线
 * 输入参数：无
 * 输出参数：无
 */
void can_config()
{
    can_parameter_struct can_parameter;     // CAN参数结构体
    can_filter_parameter_struct can_filter; // CAN滤波器参数结构体

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter); // 初始化CAN参数结构体
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);  // 初始化CAN滤波器参数结构体

    /* 初始化CAN寄存器 */
    can_deinit(CAN0);
    can_deinit(CAN1);

    /* 配置CAN参数 */
    can_parameter.time_triggered = DISABLE;           // 非时间触发模式
    can_parameter.auto_bus_off_recovery = ENABLE;     // 自动总线关闭恢复
    can_parameter.auto_wake_up = DISABLE;             // 不自动唤醒
    can_parameter.auto_retrans = ENABLE;              // 自动重传
    can_parameter.rec_fifo_overwrite = DISABLE;       // 接收FIFO不溢出
    can_parameter.trans_fifo_order = DISABLE;         // 发送FIFO不按顺序
    can_parameter.working_mode = CAN_NORMAL_MODE;     // 正常工作模式
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ; // 重新同步跳转宽度
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;    // 时间段1
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;    // 时间段2

    /* 设置波特率 */
#if CAN_BAUDRATE == 1000
    can_parameter.prescaler = 6; // 1MBps
#elif CAN_BAUDRATE == 500
    can_parameter.prescaler = 12; // 500KBps
#elif CAN_BAUDRATE == 250
    can_parameter.prescaler = 24; // 250KBps
#elif CAN_BAUDRATE == 125
    can_parameter.prescaler = 48; // 125KBps
#elif CAN_BAUDRATE == 100
    can_parameter.prescaler = 60; // 100KBps
#elif CAN_BAUDRATE == 50
    can_parameter.prescaler = 120; // 50KBps
#elif CAN_BAUDRATE == 20
    can_parameter.prescaler = 300; // 20KBps
#else
#error "please select list can baudrate in private defines in main.c "
#endif

    /* 初始化CAN */
    can_init(CAN0, &can_parameter);
    can_init(CAN1, &can_parameter);

    /* 设置CAN过滤器参数 */
    can_filter.filter_number = 0;                  // 设置过滤器编号，这里是过滤器0
    can_filter.filter_mode = CAN_FILTERMODE_MASK;  // 设置过滤器模式为屏蔽模式
    can_filter.filter_bits = CAN_FILTERBITS_32BIT; // 设置过滤器位宽为32位
    can_filter.filter_list_high = 0x0000;          // 高16位过滤器列表设置为0
    can_filter.filter_list_low = 0x0000;           // 低16位过滤器列表设置为0
    can_filter.filter_mask_high = 0x0000;          // 高16位过滤器掩码设置为0
    can_filter.filter_mask_low = 0x0000;           // 低16位过滤器掩码设置为0
    can_filter.filter_fifo_number = CAN_FIFO0;     // 设置过滤器的FIFO为FIFO0
    can_filter.filter_enable = ENABLE;             // 使能过滤器

    can_filter_init(&can_filter); // 初始化CAN过滤器

    /* CAN1过滤器编号 */
    can_filter.filter_number = 15; // 设置CAN1过滤器编号为15
    can_filter_init(&can_filter);  // 初始化CAN1过滤器
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn,0,0);

    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX0_IRQn,1,1);
}

/*!
    \brief      configure the leds
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_config(void)
{
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
    gd_eval_led_init(LED5);
}

/**

@brief CAN模块的GPIO引脚配置函数
**/
void can_gpio_config(void)
{
    /*使能CAN模块时钟*/
    rcu_periph_clock_enable(RCU_CAN0);  // 使能CAN0外设时钟。
    rcu_periph_clock_enable(RCU_CAN1);  // 使能CAN1外设时钟。
    rcu_periph_clock_enable(RCU_GPIOB); // 使能GPIOB端口时钟。
    rcu_periph_clock_enable(RCU_GPIOD); // 使能GPIOD端口时钟。
    rcu_periph_clock_enable(RCU_AF);    // 使能复用功能模块的时钟。

    /* 配置CAN0的GPIO引脚 */
    gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_0);   // 配置PD0为上拉输入模式
    gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1); // 配置PD1为复用功能推挽输出模式
    gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP, ENABLE);              // 重映射CAN0的GPIO引脚为PD0和PD1

    /* 配置CAN1的GPIO引脚 */
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);   // 配置PB5为上拉输入模式
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6); // 配置PB6为复用功能推挽输出模式
    gpio_pin_remap_config(GPIO_CAN1_REMAP, ENABLE);                   // 重映射CAN1的GPIO引脚为PB5和PB6
}

/* retarget the C library printf function to the usart */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
