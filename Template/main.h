#ifndef MAIN_H
#define MAIN_H
#include "gd32f30x_gpio.h"

#define __PACKED		__attribute__ ((__packed__))

/* 发送数据内容 */
#define SEND_DATA (0x11)

/*从节点数量*/
#define SLAVE_CONUT (31)

/* 通信超时时间：5ms */
#define COMMUNICATION_TIMEOUT (0x05)

/* 通信速率：1MBps */
#define CAN_BAUDRATE (1000)
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

/* 从节点数量 */
#define SLAVE_COUNT (32)
#define MASTER_TX_ID_START (0x51)
#define SLAVE_RX_ID_START (0x01)
#define MASTER_RX_ID_START (0x01)
#define SLAVE_TX_ID_START (0x01)

typedef struct
{
    uint8_t slave_tx_id;//从节点的发送ID
    uint8_t slave_rx_id;//从节点的过滤器ID
    uint8_t master_tx_id;//主节点的发送ID
    uint8_t master_rx_id;//主节点的过滤器ID

} __PACKED COMMUNICATION_ID_t;

typedef enum
{
    e_slave_0 = 0x00,
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
    e_master = 0xff
} ROLE_e;

typedef enum
{
    e_green_led,
    e_red_led
} LED_e;

void master_task(const uint8_t slave_index);
void slave_task(const uint8_t slave_seq);
void config_slave_communication_id(const uint8_t slave_seq);
void can_send_data(const uint8_t send_data, const uint32_t tx_sfid);
void ckeck_receive_data(void);
void can_config(void);
void init_can0_filter(void);
void config_can0_filter(uint16_t filter_id);
void nvic_config(void);
void led_on(LED_e led);
void led_off(LED_e led);
void led_twinkle(LED_e led);
void led_init(void);
void can_gpio_config(void);
void uart0_init(void);
void accurate_delay_ms(uint16_t ms);
void init_communication_id(void);
void timer3_init(uint32_t period);

#endif /* MAIN_H */

