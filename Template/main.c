#include "gd32f30x.h"
#include <stdio.h>
#include "main.h"
#include "gd32f30x_gpio.h"

/* can接收标志位 */
FlagStatus can0_receive_flag;
/* can错误标志位 */
FlagStatus can0_error_flag;
/* CAN发送参数初始化 */
static const can_trasnmit_message_struct transmit_message_init = {
    .tx_efid = 0x00,          // 初始化扩展帧ID
    .tx_ft = CAN_FT_DATA,     // 初始化发送数据帧类型
    .tx_ff = CAN_FF_STANDARD, // 初始化发送数据帧格式为标准帧
    .tx_dlen = 8,             // 初始化发送数据的长度为8
};

int main(void)
{
    // 初始化CAN0
    __can_init__();

    // 配置中断向量表
    nvic_config();

    /* 初始化串口0 */
    uart0_init();

    // 初始化LED
    led_init();

    /* 初始化CAN和过滤器 */
    can_config();

    while (1)
    {
        if (ROLE == e_master)
        {
            can_send_data(0x11, 0x51);
            receive_data();
        }
    }
}

void __can_init__()
{
    can0_receive_flag = RESET; // 将CAN0接收标志清零
    can0_error_flag = RESET;   // 将CAN0错误标志清零

    can_gpio_config(); // 配置GPIO引脚
    printf("\r\ncan init success !\r\n");
}

void can_send_data(const uint8_t send_data, const uint32_t tx_sfid)
{
    uint32_t timeout = 0xFFFF;    // 初始化发送超时时间为0xFFFF
    uint8_t transmit_mailbox = 0; // 定义发送邮箱编号为0

    can_trasnmit_message_struct transmit_message = transmit_message_init;          // 将发送消息的初始值设置为transmit_message_init
    transmit_message.tx_sfid = tx_sfid;                                            // 将发送消息的标准帧ID设置为tx_sfid
    memset(transmit_message.tx_data, send_data, sizeof(transmit_message.tx_data)); // 将发送数据的内容设置为send_data

    transmit_mailbox = can_message_transmit(CAN0, &transmit_message); // 发送CAN消息

    while (can_transmit_states(CAN0, transmit_mailbox) != CAN_TRANSMIT_OK && timeout--)
        ; // 等待发送完成

    printf("\r\ntx: ");
    for (uint8_t i = 0; i < 8; i++) // 遍历发送数据
    {
        printf("%02x ", transmit_message.tx_data[i]); // 打印每一个数据
    }
    printf("\r\n"); // 换行
}

void receive_data()
{
    can_receive_message_struct receive_message;

    /* 如果CAN0正确接收到数据，打印接收到的数据 */
    if (SET == can0_receive_flag)
    {
        can0_receive_flag = RESET;
        printf("\r\n can0 receive data:");
        for (uint8_t i = 0; i < receive_message.rx_dlen; i++)
        {
            printf(" %02x", receive_message.rx_data[i]);
        }
    }

    if (SET == can0_error_flag)
    {
        can0_error_flag = RESET;                 // 清除CAN0错误标志位
        printf("\r\n can0 communication error"); // 打印CAN0通信错误信息
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

    can_interrupt_enable(CAN0, CAN_INT_RFNE0); // 使能CAN0的接收FIFO0非空中断

    printf("\r\nconfigure can success !\r\n");
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
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    printf("\r\nconfigure CAN0 NVIC success !\r\n");
}

void led_on(LED_e led)
{
    if (led == e_red_led)
    {
        GPIO_BOP(GPIOB) = GPIO_PIN_10;
    }
    else if (led == e_green_led)
    {
        GPIO_BOP(GPIOB) = GPIO_PIN_11;
    }
}

void led_off(LED_e led)
{
    if (led == e_red_led)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_10;
    }
    else if (led == e_green_led)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_11;
    }
}

void led_init(void)
{

    rcu_periph_clock_enable(RCU_GPIOB);
    /* 初始化绿色led*/
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    led_off(e_green_led);

    /* 初始化红色led*/
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    led_off(e_red_led);

    printf("\r\nled init success !\r\n");
}

/**

@brief CAN模块的GPIO引脚配置函数
**/
void can_gpio_config(void)
{
    /*使能CAN模块时钟*/
    rcu_periph_clock_enable(RCU_CAN0);  // 使能CAN0外设时钟。
    rcu_periph_clock_enable(RCU_GPIOD); // 使能GPIOD端口时钟。
    rcu_periph_clock_enable(RCU_AF);    // 使能复用功能模块的时钟。

    /* 配置CAN0的GPIO引脚 */
    gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_0);   // 配置PD0为上拉输入模式
    gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1); // 配置PD1为复用功能推挽输出模式
    gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP, ENABLE);              // 重映射CAN0的GPIO引脚为PD0和PD1
}

/* 重映射打印函数到串口0 */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
        ;
    return ch;
}

/*!

@brief 初始化串口0

@param none

@retval none
*/
void uart0_init()
{
    /* 使能GPIO时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* 使能USART时钟 */
    rcu_periph_clock_enable(RCU_USART0);

    /*初始化发送管脚 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* 初始化接收管脚 */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART 配置 */
    usart_deinit(USART0);                                 // 将 USART0 外设的寄存器恢复到复位值，确保外设以最初的状态开始配置。
    usart_baudrate_set(USART0, 115200U);                  // 设置 USART0 的波特率为 115200。
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);   // 使能 USART0 的接收功能。
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE); // 使能 USART0 的发送功能。
    usart_enable(USART0);                                 // 使能 USART0 外设。
}