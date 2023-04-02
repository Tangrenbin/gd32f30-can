#include "gd32f30x.h"
#include <stdio.h>
#include "main.h"
#include "gd32f30x_gpio.h"

/* 角色 */
#define ROLE (e_master)
// #define ROLE (e_slave_0)
/* CAN发送参数 */
can_trasnmit_message_struct transmit_message;
/* CAN接收参数 */
can_receive_message_struct receive_message;
/* CAN滤波器参数 */
can_filter_parameter_struct can_filter;
/* 通信ID */
COMMUNICATION_ID_t communication_id[SLAVE_COUNT];

/* 确认帧格式 */
const uint8_t sack_frame [8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
/* 数据帧格式 */
const uint8_t data_frame [8] = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
/* 从节点序号 */
uint8_t g_slave_index = 0;
/* 主节点回复确认帧标志 */
uint8_t g_master_ack_flag = RESET;

int main(void)
{
    /* 配置GPIO引脚 */
    can_gpio_config();

    /* 初始化can接收参数 */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);

    /* 初始化can发送参数  */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);

    /* 配置中断向量表 */
    nvic_config();

    /* 初始化串口0 */
    uart0_init();

    /* 初始化LED */
    led_init();

    /* 初始化CAN和过滤器 */
    can_config();
    
    /* 初始化通信id */
    init_communication_id();

    if (ROLE == e_master)
    {
        /* 配置定时器3 */
        timer3_init(5000); // 定时器3每5000（5秒）个计数周期产生一次中断进行主节点周期发送任务
    }
    else
    {
        slave_task(ROLE);
    }

    while (1)
    {

    }
}

/**

@brief 主任务函数，向指定从节点发送数据并配置CAN过滤器

@param[in] slave_index 指定从节点的索引

@note 数据发送标识符为指定从节点的主收标识符，数据内容为固定的11 11 11 11 11 11 11 11

@note 数据发送后会收到从节点的确认帧，主节点需要回复确认帧

@note 确认帧数据内容格式固定为：FF FF FF FF FF FF FF FF，长度固定为8个字节

@note 主节点灯语：接收数据时闪烁绿色LED，发送数据时闪烁红色LED

@note 主节点发送数据后会配置CAN过滤器以接收从节点的确认帧

@note 主节点会按照从节点的索引依次发送数据，当索引达到最大值时重新从0开始循环
*/
void master_task(const uint8_t slave_index)
{
    // 向指定从节点发送数据
    can_send_data(SEND_DATA, communication_id[slave_index].master_tx_id);

    // 配置CAN过滤器以接收从节点的确认帧
    config_can0_filter(communication_id[slave_index].master_rx_id);

    // 更新下一个从节点的索引
    g_slave_index = (slave_index + 1) % SLAVE_COUNT;
}

void slave_task(const uint8_t slave_seq)
{
    config_slave_communication_id(slave_seq);
}

void config_slave_communication_id(const uint8_t slave_seq)
{
    // 配置CAN过滤器以接收主节点的确认帧
    if (slave_seq < SLAVE_COUNT)
    {
        config_can0_filter(communication_id[slave_seq].slave_rx_id);
    }
}


void can_send_data(const uint8_t send_data, const uint32_t tx_sfid)
{
    uint32_t timeout = 0xFFFF;    // 初始化发送超时时间为0xFFFF
    uint8_t transmit_mailbox = 0; // 定义发送邮箱编号为0

    transmit_message.tx_sfid = tx_sfid;                                            // 将发送消息的标准帧ID设置为tx_sfid
    memset(transmit_message.tx_data, send_data, sizeof(transmit_message.tx_data)); // 将发送数据的内容设置为send_data

    transmit_mailbox = can_message_transmit(CAN0, &transmit_message); // 发送CAN消息

    while (can_transmit_states(CAN0, transmit_mailbox) != CAN_TRANSMIT_OK && timeout--)
        ; // 等待发送完成

    led_twinkle(e_red_led);

    printf("\r\ntx [%02x]: ",transmit_message.tx_sfid);
    for (uint8_t i = 0; i < transmit_message.tx_dlen; i++) // 遍历发送数据
    {
        printf("%02x ", transmit_message.tx_data[i]); // 打印每一个数据
    }
    printf("\r\n"); // 换行
}

void ckeck_receive_data(void)
{
    if (ROLE == e_master)
    {
        if ((communication_id[g_slave_index].master_rx_id == receive_message.rx_sfid) &&
            (CAN_FF_STANDARD == receive_message.rx_ff) &&
            (8 == receive_message.rx_dlen) &&
            (0 == memcmp(receive_message.rx_data, sack_frame, sizeof(sack_frame))))
        {
            printf("\r\ncheck receive data success , start send sack frame to slave !\r\n");
            can_send_data(0xff, communication_id[g_slave_index].master_tx_id); // 回复确认帧
        }
        else
        {
            printf("\r\ncheck receive data fail !\r\n");
        }
    }
    else if (ROLE < SLAVE_COUNT)
    {
        // 合法数据
        if ((receive_message.rx_sfid == communication_id[ROLE].slave_rx_id) &&
            (CAN_FF_STANDARD == receive_message.rx_ff) &&
            (8 == receive_message.rx_dlen))

        {
            if (0 == memcmp(receive_message.rx_data, data_frame, sizeof(data_frame))) // 数据帧
            {
                printf("\r\ncheck receive data success , start send sack frame to master !\r\n");
                can_send_data(0xff, communication_id[ROLE].slave_tx_id); // 回复确认帧
                timer3_init(5);                                          // 开启5毫秒超时检测
                led_off(e_red_led);
            }
            else if (0 == memcmp(receive_message.rx_data, sack_frame, sizeof(sack_frame))) // 确认帧
            {
                can_send_data(0xff, communication_id[ROLE].slave_tx_id); // 回复确认帧
                g_master_ack_flag = SET;
            }
            else
            {
                g_master_ack_flag = RESET;
            }
        }
        else
        {
            printf("\r\ncheck receive data fail !\r\n");
        }
    }
}
/*
 * 函数名：can_config
 * 功能描述：配置CAN总线
 * 输入参数：无
 * 输出参数：无
 */
void can_config(void)
{
    can_parameter_struct can_parameter;     // CAN参数结构体   

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter); // 初始化CAN参数结构体

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

    init_can0_filter(); 

    can_interrupt_enable(CAN0, CAN_INT_RFNE0); // 使能CAN0的接收FIFO0非空中断

    printf("\r\nconfigure can success !\r\n");
}

void init_can0_filter(void)
{
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter); // 初始化CAN滤波器参数结构体
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

    printf("\r\n init can0 filter success !\r\n");
}

void config_can0_filter(uint16_t filter_id)
{
    can_filter.filter_list_high = 0x0000;   // 过滤器列表高位为空
    can_filter.filter_list_low = filter_id; // 过滤器列表低位为0x01
    can_filter.filter_mask_high = 0x0000;   // 掩码高位为0
    can_filter.filter_mask_low = 0xFF00;    // 掩码低位为0xFF00，表示只匹配ID低8位

    printf("\r\n config filter id : %02x  !\r\n", can_filter.filter_list_low);
}
/*
!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    /* 配置CAN0 NVIC */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 0, 0);
    printf("\r\nCAN0 NVIC config success !\r\n");

    /* 配置定时器3 NVIC */
    nvic_irq_enable(TIMER3_IRQn, 1, 0);
    printf("\r\ntimer3 NVIC config success !\r\n");
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

void led_twinkle(LED_e led)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        led_on(led);
        accurate_delay_ms(200);
        led_off(led);
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
    /* 使能CAN0和GPIOB时钟 */
    rcu_periph_clock_enable(RCU_CAN0);  
    rcu_periph_clock_enable(RCU_GPIOB); 

    /* 配置PB8和PB9为CAN0的RX和TX引脚 */
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);  // RX
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9); // TX
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
void uart0_init(void)
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

/**

@brief 该函数使用了定时器2来实现毫秒级延迟，同时不会影响CAN0中断。函数的参数ms可以指定延迟的毫秒数，最大延迟时间为6553.5毫秒

@param ms: 延时的毫秒数

@retval None
**/
void accurate_delay_ms(uint16_t ms)
{
    rcu_periph_clock_enable(RCU_TIMER2); // 使能定时器 2 时钟

    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara); // 使用默认值初始化定时器配置

    timer_initpara.prescaler = SystemCoreClock / 10000 - 1; // 定时器 2 时钟频率为系统时钟频率的 1/10000
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = ms * 10;     // 定时器 2 的周期为需要延时的毫秒数乘以 10（因为时钟频率为 1/10000）
    timer_init(TIMER2, &timer_initpara); // 初始化定时器 2

    timer_flag_clear(TIMER2, TIMER_FLAG_UP); // 清除定时器 2 的计数标志位
    timer_enable(TIMER2);                    // 启动定时器 2

    while (!timer_flag_get(TIMER2, TIMER_FLAG_UP))
    {
    } // 等待定时器 2 计数到 0

    timer_disable(TIMER2);                // 停止定时器 2
    rcu_periph_clock_disable(RCU_TIMER2); // 关闭定时器 2 时钟
}

/*!
\brief CAN0接收中断函数
\param[in] 无
\param[out] 无
\retval 无
*/

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    can_message_receive(CAN0, CAN_FIFO0, &receive_message); // 读取CAN0的接收缓存FIFO0中的消息到结构体receive_message中

    led_twinkle(e_green_led); // 绿色指示灯闪烁

    ckeck_receive_data();//判断是否符合通信协议

    printf("\r\nrx [%02x]: ", receive_message.rx_sfid);
    for (uint8_t i = 0; i < receive_message.rx_dlen; i++) // 遍历发送数据
    {
        printf("%02x ", receive_message.rx_data[i]); // 打印每一个数据
    }
    printf("\r\n"); // 换行
}

void init_communication_id(void)
{
    for (uint8_t i = 0; i < SLAVE_COUNT; i++)
    {
        communication_id[i].slave_tx_id = SLAVE_TX_ID_START + i;
        communication_id[i].slave_rx_id = SLAVE_RX_ID_START + i;
        communication_id[i].master_tx_id = MASTER_TX_ID_START + i;
        communication_id[i].master_rx_id = MASTER_RX_ID_START + i;
    }
}

void TIMER3_IRQHandler(void)
{
    if (timer_flag_get(TIMER3, TIMER_FLAG_UP) == SET)
    {
        if (ROLE == e_master) // 主节点
        {
            timer_flag_clear(TIMER3, TIMER_FLAG_UP);
            master_task(g_slave_index); // 每5秒调用一次周期任务
        }
        else // 从节点
        {
            /* 从节点超时检测 */
            if (g_master_ack_flag == RESET)
            {
                led_on(e_red_led); // 超时，点亮红灯
                printf("\r\n[%d]: wait master sack time out !\r\n", ROLE);
            }
        }
    }
}

void timer3_init(uint32_t period)
{
    /* 使能定时器3时钟 */
    rcu_periph_clock_enable(RCU_TIMER3);

    /* 定时器3的基本配置 */
    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = SystemCoreClock / 10000 - 1;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = period * 10;  // 每period毫秒触发一次中断
    timer_init(TIMER3, &timer_initpara);

    /* 使能定时器3中断 */
    timer_interrupt_enable(TIMER3, TIMER_INT_UP);

    /* 启动定时器3 */
    timer_enable(TIMER3);
}
