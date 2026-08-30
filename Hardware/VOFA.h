#ifndef __VOFA_H__
#define __VOFA_H__

#define VOFA_USART          USART1                      // VOFA使用的串口
#define VOFA_RCC            RCC_APB2Periph_USART1       // VOFA使用的串口时钟
#define VOFA_GPIO_RCC       RCC_AHB1Periph_GPIOB        // VOFA使用的串口GPIO时钟
#define VOFA_GPIO_PORT      GPIOB                       // VOFA使用的串口GPIO端口
#define VOFA_GPIO_TX_PIN    GPIO_Pin_6                  // VOFA使用的串口GPIO发送引脚
#define VOFA_GPIO_RX_PIN    GPIO_Pin_7                  // VOFA使用的串口GPIO接收引脚
#define VOFA_TX_PINSOURCE   GPIO_PinSource6             // VOFA使用的串口GPIO发送引脚源
#define VOFA_RX_PINSOURCE   GPIO_PinSource7             // VOFA使用的串口GPIO接收引脚源
#define VOFA_AF             GPIO_AF_USART1              // VOFA使用的串口GPIO复用功能
#define VOFA_ISR            USART1_IRQHandler           // VOFA使用的串口中断函数名
#define VOFA_IRQn           USART1_IRQn                 // VOFA使用的串口中断通道号
#define VOFA_IRQ_DISABLE()  NVIC_DisableIRQ(VOFA_IRQn)  // 禁止VOFA中断
#define VOFA_IRQ_ENABLE()   NVIC_EnableIRQ(VOFA_IRQn)   // 使能VOFA中断
#define VOFA_Timer          TIM6                        // VOFA超时检测使用的定时器
#define VOFA_Timer_RCC      RCC_APB1Periph_TIM6         // VOFA超时检测使用的定时器时钟
#define VOFA_TIM_PERIOD     (VOFA_RX_TIMEOUT * 1000)    // VOFA超时检测使用的定时器周期
#define VOFA_TIM_PRESCALER  84                          // VOFA超时检测使用的定时器预分频值

#define VOFA_BIG_ENDIAN     0                           // 0=小端（STM32等Cortex-M，无需交换）, 1=大端（如Keil C51的8051/STC），发送前自动交换字节序
#define VOFA_RX_BUF_SIZE    256                         // VOFA接收缓冲区大小,其必须为2 的幂
#define VOFA_MAX_CHANNELS   16                          // VOFA最大通道数

#define VOFA_RX_HEAD0       0xAA                        // VOFA接收数据包头标志1
#define VOFA_RX_HEAD1       0x55                        // VOFA接收数据包头标志2
#define VOFA_RX_TAIL0       0x0D                        // VOFA接收数据包固定帧尾1
#define VOFA_RX_TAIL1       0x0A                        // VOFA接收数据包固定帧尾2
#define VOFA_RX_MAX_DATA    8                           // VOFA接收数据的最大数据长度
#define VOFA_RX_OVERHEAD    6                           // 帧头2字节 + 长度1字节 + 命令1字节 + 帧尾2字节
#define VOFA_RX_TIMEOUT     50                          // VOFA接收数据包超时时间（单位：ms）

#define CMD_SET_KP          0x01                        // 设置PID参数Kp
#define CMD_SET_KI          0x02                        // 设置PID参数Ki
#define CMD_SET_KD          0x03                        // 设置PID参数Kd
#define CMD_SET_SPEED       0x04                        // 设置电机速度
#define CMD_SET_MODE        0x05                        // 设置工作模式
#define CMD_SET_STOP        0x06                        // 设置停止命令
#define CMD_SET_START       0x07                        // 设置启动命令

typedef union                                           //联合体
{
    float f;
    uint8_t c[4];
} FLOAT_CONV;

typedef struct
{
    uint8_t buf[VOFA_RX_BUF_SIZE];                      // 接收缓冲区
    volatile uint16_t head;                             // 缓冲区头指针
    volatile uint16_t tail;                             // 缓冲区尾指针
} VOFA_RxBuffer;

typedef struct
{
    uint8_t state;                                      // 数据包接收状态,0=等AA, 1=等55, 2=收数据域
    uint8_t buf[VOFA_RX_OVERHEAD + VOFA_RX_MAX_DATA];   // AA 55 + LEN + CMD + PAYLOAD + 0D 0A
    uint8_t idx;                                        // 数据包接收索引
    uint16_t data_len;                                  // PAYLOAD长度，不包含帧头、LEN、CMD和固定帧尾
    uint8_t frame_ok;                                   // 数据包接收完成标志
} VOFA_RxFrame;

typedef enum
{
    VOFA_CMD_RESULT_NONE = 0,                           // 无命令执行结果
    VOFA_CMD_RESULT_OK,                                 // 命令执行成功
    VOFA_CMD_RESULT_INVALID_LENGTH,                     // 无效命令
    VOFA_CMD_RESULT_INVALID_COMMAND,                    // 无效命令
    VOFA_CMD_RESULT_INVALID_VALUE,                      // 无效数据
    VOFA_CMD_RESULT_UNKNOWN_ERROR                       // 未知错误
} VOFA_CmdResult;

typedef struct
{
    uint8_t last_command;                               // 上一次接收到的命令字节
    VOFA_CmdResult last_result;                         // 上一次命令的处理结果
    uint32_t total_count;                               // 通过帧结构检查并进入命令分发的总帧数
    uint32_t success_count;                             // 成功处理命令计数
    uint32_t invalid_length_count;                      // 无效命令长度计数
    uint32_t invalid_command_count;                     // 无效命令计数
    uint32_t invalid_value_count;                       // 无效数据值计数
    uint32_t unknown_error_count;                       // 未知错误计数
} VOFA_CmdStatus;


/*********************************函数声明***********************************/
void VOFA_Init(uint32_t bound);
void VOFA(float *data, uint16_t num);
void VOFA_Process(void);
const VOFA_CmdStatus* VOFA_GetCmdStatus(void);

/*******************************全局变量声明*********************************/

#endif
