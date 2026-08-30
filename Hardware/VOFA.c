#include "stm32f4xx.h"                  // Device header
#include "string.h"
#include "math.h"
#include "VOFA.h"

static VOFA_RxBuffer VOFA_rx_buffer;           // VOFA接收缓冲区初始化
static VOFA_RxFrame VOFA_rx_frame;             // VOFA接收数据包初始化
static VOFA_CmdStatus VOFA_cmd_status;         // VOFA命令状态结构体
static volatile uint8_t VOFA_rx_fault = 0;     // VOFA接收故障标志
// 接收帧: AA 55 | LEN | CMD | PAYLOAD | 0D 0A；LEN仅表示PAYLOAD长度，整帧长度为LEN+6

/**
  *  @brief  将浮点数由大端存储改为小端存储或是将浮点数由小端存储改为大端存储
  *  @param  需转换的浮点数
  *  @retval 转换后的浮点数
  *  @memo   内部使用
  **/
  
#if VOFA_BIG_ENDIAN
static float BLEndianFloat(float fValue)
{
    FLOAT_CONV d1, d2;
    d1.f = fValue;
    d2.c[0] = d1.c[3];
    d2.c[1] = d1.c[2];
    d2.c[2] = d1.c[1];
    d2.c[3] = d1.c[0];
    return d2.f;
}
#endif


/**
  *  @brief  初始化VOFA定时器
  *  @param  无
  *  @retval 无
  *  @memo   无
  **/
  
static void VOFA_TimeInit(void)
{
    RCC_APB1PeriphClockCmd(VOFA_Timer_RCC, ENABLE);                                        // 使能TIM6时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                                // 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                            // 向上计数模式
    TIM_TimeBaseStructure.TIM_Period = VOFA_TIM_PERIOD - 1;                                // 自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = VOFA_TIM_PRESCALER - 1;                          // 时钟预分频数
    TIM_TimeBaseInit(VOFA_Timer, &TIM_TimeBaseStructure);

    TIM_ClearFlag(VOFA_Timer, TIM_FLAG_Update);                                            // 清溢出标志
    TIM_SetCounter(VOFA_Timer, 0);                                                         // 计数器从 0 开始
    TIM_Cmd(VOFA_Timer, ENABLE);                                                           // 使能TIM6
}

/**
  *  @brief  初始化VOFA串口
  *  @param  bound: 串口波特率
  *  @retval 无
  *  @memo   在头文件中修改相应串口宏定义
  **/
  
void VOFA_Init(uint32_t bound)
{
    if (VOFA_USART == USART1 || VOFA_USART == USART6)
        RCC_APB2PeriphClockCmd(VOFA_RCC, ENABLE);
    else
        RCC_APB1PeriphClockCmd(VOFA_RCC, ENABLE);
	RCC_AHB1PeriphClockCmd(VOFA_GPIO_RCC, ENABLE);

    VOFA_TimeInit();                                                                            // 初始化定时器，用于接收超时判断
	
    GPIO_PinAFConfig(VOFA_GPIO_PORT, VOFA_TX_PINSOURCE, VOFA_AF);
    GPIO_PinAFConfig(VOFA_GPIO_PORT, VOFA_RX_PINSOURCE, VOFA_AF);

	GPIO_InitTypeDef GPIO_InitStructure;                        //PB6 - TX, PB7 - RX, 串口1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = VOFA_GPIO_TX_PIN | VOFA_GPIO_RX_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(VOFA_GPIO_PORT, &GPIO_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = VOFA_IRQn;                                             // 串口中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	USART_InitTypeDef USART_InitStructure;
 	USART_InitStructure.USART_BaudRate = bound;                                       			// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						            // 8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							            // 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								            // 奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	  			// 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				            	// 收发模式
	USART_Init(VOFA_USART, &USART_InitStructure);

	USART_ITConfig(VOFA_USART, USART_IT_RXNE, ENABLE);											// 开启中断
	USART_Cmd(VOFA_USART, ENABLE);                    											// 使能串口
}

/**
  *  @brief  无线串口发送数据
  *  @param  需要发送的数据地址, 需要发送的长度
  *  @retval 无
  *  @memo   无
  **/
  
static void VOFA_Sent_Data(uint8_t *buf, uint16_t len)
{
	uint16_t temp;
  	for(temp = 0;temp < len; temp++)		// 循环发送数据
	{		   
		while(USART_GetFlagStatus(VOFA_USART, USART_FLAG_TXE) == RESET);
		USART_SendData(VOFA_USART, buf[temp]);
	}
	while(USART_GetFlagStatus(VOFA_USART, USART_FLAG_TC) == RESET);	
}

/**
  *  @brief  VOFA集成函数
  *  @param  浮点数组指针, num: 数据路数
  *  @retval 无
  *  @memo   无
  **/
  
void VOFA(float *data, uint16_t num)
{
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    if (data == NULL || num == 0 || num > VOFA_MAX_CHANNELS)
        return;
#if VOFA_BIG_ENDIAN
    uint16_t i;
    float v;
    for (i = 0; i < num; i++)
    {
        v = BLEndianFloat(data[i]);                                                             // 大端内存→小端字节序
        VOFA_Sent_Data((uint8_t*)&v, sizeof(v));
    }
#else
    VOFA_Sent_Data((uint8_t*)data, sizeof(float) * num);                                        // 小端直接发
#endif
    VOFA_Sent_Data(tail, sizeof(tail));
}

/**
  * @brief  复位接收解析状态机（不清空环形缓冲区）
  * @param  无
  * @retval 无
  * @memo   接收出错或环形缓冲满时，只作废当前正在解析的帧，
  *          缓冲区中已缓存的其他数据仍会继续解析
  */
  
static void VOFA_ResetRxFrame(void)
{
    VOFA_rx_frame.state = 0;                                                                    // 重置接收状态机
    VOFA_rx_frame.idx = 0;                                                                      // 重置接收索引
    VOFA_rx_frame.data_len = 0;                                                                 // 清除数据长度
    VOFA_rx_frame.frame_ok = 0;                                                                 // 清除接收完成标志
}

/**
  *  @brief  解析VOFA接收数据
  *  @param  接收到的单个字节数据
  *  @retval 无
  *  @memo   无
  **/
  
static void VOFA_Unpack(uint8_t data)
{
    switch (VOFA_rx_frame.state)
    {
        case 0:                                                                                 // 等待数据包头标志1（0xAA）
            if (data == VOFA_RX_HEAD0)
            {
                VOFA_rx_frame.buf[0] = data;
                VOFA_rx_frame.idx = 1;
                VOFA_rx_frame.state = 1;
            }
            break;
        case 1:                                                                                 // 等待数据包头标志2（0x55）
            if (data == VOFA_RX_HEAD1)
            {
                VOFA_rx_frame.buf[1] = data;
                VOFA_rx_frame.idx = 2;
                VOFA_rx_frame.state = 2;
            }
            else if (data != VOFA_RX_HEAD0)                                                     //既非0x55也非新0xAA → 复位,若是0xAA则保持 state=1，继续等0x55
                VOFA_rx_frame.state = 0;
            break;
        case 2:                                                                                 // 接收数据域
            if (VOFA_rx_frame.idx >= sizeof(VOFA_rx_frame.buf))
            {
                VOFA_rx_frame.state = 0;
                break;
            }
            VOFA_rx_frame.buf[VOFA_rx_frame.idx++] = data;
            if (VOFA_rx_frame.idx == 3)
            {
                VOFA_rx_frame.data_len = VOFA_rx_frame.buf[2];                                  // LEN仅表示PAYLOAD长度
                if (VOFA_rx_frame.data_len > VOFA_RX_MAX_DATA)
                {
                    VOFA_ResetRxFrame();
                    break;
                }
            }
            if (VOFA_rx_frame.idx == VOFA_rx_frame.data_len + VOFA_RX_OVERHEAD)
            {
                uint16_t tail_idx = VOFA_rx_frame.data_len + 4;
                if ((VOFA_rx_frame.buf[tail_idx] == VOFA_RX_TAIL0) &&
                    (VOFA_rx_frame.buf[tail_idx + 1] == VOFA_RX_TAIL1))
                {
                    VOFA_rx_frame.frame_ok = 1;                                                 // 帧长和固定帧尾均正确
                }
                VOFA_rx_frame.state = 0;
            }
            break;
        default:
            VOFA_rx_frame.state = 0;
            break;
    }
}

/**
  * @brief  从小端PAYLOAD中读取float
  * @param  payload: PAYLOAD起始地址
  * @param  payload_len: PAYLOAD长度
  * @param  value: 解析结果
  * @retval 1=成功，0=数据无效
  */
  
static uint8_t VOFA_ReadFloat(const uint8_t *payload, uint16_t payload_len, float *value)
{
    if (payload == NULL || value == NULL)
        return 0;
    if (payload_len != sizeof(float))
        return 0;
#if VOFA_BIG_ENDIAN
    {
        uint8_t reversed[sizeof(float)];
        uint8_t i;
        for (i = 0; i < sizeof(float); i++)
        {
            reversed[i] = payload[sizeof(float) - 1 - i];
        }
        memcpy(value, reversed, sizeof(float));
    }
#else
    memcpy(value, payload, sizeof(float));
#endif
    if (!isfinite(*value)) // 检查是否为有限数
    {
        return 0; // 如果不是有限数，返回错误
    }
    return 1;
}


/**
  *  @brief  检查浮点数是否在有效范围内
  *  @param  value: 要检查的浮点数
  *  @param  min_value: 最小值
  *  @param  max_value: 最大值
  *  @retval 1=在范围内，0=不在范围内
  *  @memo
  **/
  
static uint8_t VOFA_IsFloatInRange(float value, float min_value, float max_value)
{
    return (value >= min_value) && (value <= max_value);
}


/**
  *  @brief  分发处理接收到的数据
  *  @param  command: 命令字节
  *  @param  payload: PAYLOAD起始地址
  *  @param  payload_len: PAYLOAD长度
  *  @retval 命令处理结果
  *  @memo
  **/

static VOFA_CmdResult VOFA_DispatchData(uint8_t command, const uint8_t *payload, uint16_t payload_len)
{
    float value;
    switch (command)
    {
        case CMD_SET_KP:
            if (!VOFA_ReadFloat(payload, payload_len, &value))
                return (payload_len != sizeof(float)) ? VOFA_CMD_RESULT_INVALID_LENGTH : VOFA_CMD_RESULT_INVALID_VALUE;
            if (!VOFA_IsFloatInRange(value, 0.0f, 100.0f))
                return VOFA_CMD_RESULT_INVALID_VALUE;
            // 处理接收到的浮点数值
            return VOFA_CMD_RESULT_OK;
        case CMD_SET_KI:
            if (!VOFA_ReadFloat(payload, payload_len, &value))
                return (payload_len != sizeof(float)) ? VOFA_CMD_RESULT_INVALID_LENGTH : VOFA_CMD_RESULT_INVALID_VALUE;
            if (!VOFA_IsFloatInRange(value, 0.0f, 100.0f))
                return VOFA_CMD_RESULT_INVALID_VALUE;
            // 处理接收到的浮点数值
            return VOFA_CMD_RESULT_OK;
        case CMD_SET_KD:
            if (!VOFA_ReadFloat(payload, payload_len, &value))
                return (payload_len != sizeof(float)) ? VOFA_CMD_RESULT_INVALID_LENGTH : VOFA_CMD_RESULT_INVALID_VALUE;
            if (!VOFA_IsFloatInRange(value, 0.0f, 100.0f))
                return VOFA_CMD_RESULT_INVALID_VALUE;
            // 处理接收到的浮点数值
            return VOFA_CMD_RESULT_OK;
        case CMD_SET_SPEED:
            if (!VOFA_ReadFloat(payload, payload_len, &value))
                return (payload_len != sizeof(float)) ? VOFA_CMD_RESULT_INVALID_LENGTH : VOFA_CMD_RESULT_INVALID_VALUE;
            if (!VOFA_IsFloatInRange(value, -1000.0f, 1000.0f))
                return VOFA_CMD_RESULT_INVALID_VALUE;
            // 处理接收到的浮点数值
            return VOFA_CMD_RESULT_OK;
        case CMD_SET_MODE:
            if (payload_len != 1)
                return VOFA_CMD_RESULT_INVALID_LENGTH;
            if (payload[0] > 9) // 假设模式值只能是0-9
                return VOFA_CMD_RESULT_INVALID_VALUE;
            // 处理接收到的模式值
            return VOFA_CMD_RESULT_OK;
        case CMD_SET_STOP:
            if (payload_len != 0)
                return VOFA_CMD_RESULT_INVALID_LENGTH;
            // 处理接收到的模式设置命令
            return VOFA_CMD_RESULT_OK;
        case CMD_SET_START:
            if (payload_len != 0)
                return VOFA_CMD_RESULT_INVALID_LENGTH;
            /*
             * 启动前还应检查：
             * 1. 系统是否存在故障
             * 2. 母线电压是否正常
             * 3. 电流采样是否完成校准
             * 4. PWM输出条件是否满足
             */
            // 处理停止或启动命令
            return VOFA_CMD_RESULT_OK;
        default:
            return VOFA_CMD_RESULT_INVALID_COMMAND;
    }
}

/**
  *  @brief  记录命令执行结果
  *  @param  command: 命令字节
  *  @param  result: 命令执行结果
  *  @retval 无
  *  @memo   无
  **/

static void VOFA_RecordCmdResult(uint8_t command, VOFA_CmdResult result)
{
    VOFA_cmd_status.last_command = command;
    VOFA_cmd_status.last_result = result;
    VOFA_cmd_status.total_count++;
    // 根据result处理命令执行结果，例如发送响应或记录日志
    switch (result)
    {
        case VOFA_CMD_RESULT_OK:
            VOFA_cmd_status.success_count++;                                                    // 成功处理命令
            break;
        case VOFA_CMD_RESULT_INVALID_LENGTH:
            VOFA_cmd_status.invalid_length_count++;                                             // 无效命令长度
            break;
        case VOFA_CMD_RESULT_INVALID_COMMAND:
            VOFA_cmd_status.invalid_command_count++;                                            // 无效命令
            break;
        case VOFA_CMD_RESULT_INVALID_VALUE:
            VOFA_cmd_status.invalid_value_count++;                                              // 无效数据值
            break;
        case VOFA_CMD_RESULT_UNKNOWN_ERROR:
            VOFA_cmd_status.unknown_error_count++;                                              // 未知错误
            break;
        default:
            VOFA_cmd_status.unknown_error_count++;                                              // 其他未定义的结果处理
            break;
    }
}

/**
  * @brief  消费串口环形缓冲区并处理完整数据包
  * @param  无
  * @retval 无
  * @memo   应在主循环中高频调用，不要放在USART中断中执行
  */
  
void VOFA_Process(void)                                                                         //ISR只修改head，主循环只修改tail
{
    if (TIM_GetFlagStatus(VOFA_Timer, TIM_FLAG_Update) != RESET)                                // 若定时器溢出
    {
        VOFA_IRQ_DISABLE();                                                                     // 禁止中断，防止ISR修改缓冲区
        if (TIM_GetFlagStatus(VOFA_Timer, TIM_FLAG_Update) != RESET)                            // 再次检查定时器溢出标志，防止在禁止中断后又发生溢出
        {
            TIM_ClearFlag(VOFA_Timer, TIM_FLAG_Update);                                         // 清除定时器溢出标志
            if (VOFA_rx_frame.state != 0)                                                       // 若接收状态机不在初始状态
            {
                VOFA_ResetRxFrame();                                                            // 重置接收状态机
            }
        }
        VOFA_IRQ_ENABLE();                                                                      // 重新使能中断
    }
    if (VOFA_rx_fault)                                                                          // 若接收故障
    {
        VOFA_IRQ_DISABLE();                                                                     // 禁止中断，防止ISR修改缓冲区
        VOFA_rx_fault = 0;                                                                      // 清除接收故障标志
        VOFA_ResetRxFrame();                                                                    // 重置接收状态机
        VOFA_IRQ_ENABLE();                                                                      // 重新使能中断
    }
    while (VOFA_rx_buffer.head != VOFA_rx_buffer.tail)                                          // 若缓冲区不为空
    {
        uint8_t data = VOFA_rx_buffer.buf[VOFA_rx_buffer.tail];                                 // 读取缓冲区数据
        VOFA_rx_buffer.tail = (VOFA_rx_buffer.tail + 1) & (VOFA_RX_BUF_SIZE - 1);               // 更新尾指针
        VOFA_Unpack(data);                                                                      // 解析数据
        if (VOFA_rx_fault)
            break;                                                                              // 若接收故障，退出循环，等待下一次调用
        if (VOFA_rx_frame.frame_ok)                                                             // 若接收完成
        {
            VOFA_rx_frame.frame_ok = 0;                                                         // 清除标志位,避免业务函数提前返回或发生错误后，下一次调用VOFA_Process()重复执行同一命令
            uint8_t command = VOFA_rx_frame.buf[3];                                             // 获取命令字节
            uint8_t *payload = &VOFA_rx_frame.buf[4];                                           // 获取负载数据
            uint16_t payload_len = VOFA_rx_frame.data_len;                                      // 获取负载长度
            VOFA_CmdResult result = VOFA_DispatchData(command, payload, payload_len);           // 分发数据
            VOFA_RecordCmdResult(command, result);                                              // 处理命令执行结果
        }
    }
}

/**
  *  @brief  获取VOFA命令状态结构体指针
  *  @param  无
  *  @retval VOFA_CmdStatus结构体指针
  *  @memo   该函数返回的指针指向的是静态变量，主函数中应避免直接修改其内容，使用方式：const VOFA_CmdStatus* cmd_status = VOFA_GetCmdStatus();
  **/
  
const VOFA_CmdStatus* VOFA_GetCmdStatus(void)
{
    return &VOFA_cmd_status;
}

/**
  *  @brief  VOFA串口中断服务函数
  *  @param  无
  *  @retval 无
  *  @memo   无
  **/
  
void VOFA_ISR(void)
{
    if (VOFA_USART -> SR & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))    //当前字节不可信，直接丢弃
    {
        (void)VOFA_USART -> DR;                                                                 //读DR清除溢出标志
        VOFA_rx_fault = 1;                                                                      //设置接收故障标志
        return;
    }
    if (USART_GetITStatus(VOFA_USART, USART_IT_RXNE) != RESET)                                  //接收中断
    {
        uint8_t data = (uint8_t)USART_ReceiveData(VOFA_USART);                                  //读取接收到的数据
        uint16_t next_head = (VOFA_rx_buffer.head + 1) & (VOFA_RX_BUF_SIZE - 1);                //计算下一个头指针位置，& (VOFA_RX_BUF_SIZE - 1) 等价于 % VOFA_RX_BUF_SIZE
        if (next_head != VOFA_rx_buffer.tail)                                                   //检查缓冲区是否已满
        {
            VOFA_rx_buffer.buf[VOFA_rx_buffer.head] = data;                                     //将数据存入缓冲区
            VOFA_rx_buffer.head = next_head;                                                    //更新头指针
            TIM_SetCounter(VOFA_Timer, 0);                                                      //接收数据时，重置定时器计数器，用于超时判断
            TIM_ClearFlag(VOFA_Timer, TIM_FLAG_Update);                                         //清除定时器溢出标志
        }                                                                                       //满则丢弃，不覆盖未读数据
        else
            VOFA_rx_fault = 1;                                                                  //设置接收故障标志
    }
}
