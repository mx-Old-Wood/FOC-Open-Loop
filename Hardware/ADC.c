#include "stm32f4xx.h" // Device header
#include "Delay.h"
#include "ADC.h"

typedef struct
{
    uint32_t GPIO_RCC;
    GPIO_TypeDef *GPIO_Port;
    uint32_t GPIO_Pin;
} ADC_GPIO;

static const ADC_GPIO ADC_GPIO_List[ADC_CHANNEL_COUNT] =
    {
        [I_U] = {RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_5},
        [Temp] = {RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_4},
        [Ubus] = {RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_0},
        [I_W] = {RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_1},
        [POT] = {RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_4},
};

static volatile uint16_t AD_Value[ADC_CHANNEL_COUNT];

static void AD_DMAInit(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;                                      // DMA2数据流0的请求通道选择,此处为通道0
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC_USED -> DR;               // 外设站点基地址(32位地址),指向ADC数据寄存器
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;         // 外设站点数据宽度,当前以半字(16位)进行传输
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                    // 外设站点地址是否自增,此处不自增,始终读取ADC数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr  = (uint32_t)AD_Value;                        // 存储器0站点基地址,指向ADC采样缓冲区
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;                 // 存储器站点数据宽度,当前以半字(16位)进行传输
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                             // 存储器站点地址是否自增,此处使用地址自增
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                             // 指定传输方向,此处由外设传输到存储器
    DMA_InitStructure.DMA_BufferSize = ADC_CHANNEL_COUNT;                               // 传输数据单元数量,此处为ADC通道数(单位为半字)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                     // 传输模式,此处为循环模式,搬完一组后从缓冲区开头重新写入
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                                 // DMA数据流软件优先级,此处为高优先级

    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);
}

void AD_Init(void)
{
    uint32_t GPIO_Rcc = 0;
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        GPIO_Rcc |= ADC_GPIO_List[i].GPIO_RCC;
    RCC_AHB1PeriphClockCmd(GPIO_Rcc, ENABLE);
    RCC_APB2PeriphClockCmd(ADC_USED_RCC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        GPIO_InitStructure.GPIO_Pin = ADC_GPIO_List[i].GPIO_Pin;
        GPIO_Init(ADC_GPIO_List[i].GPIO_Port, &GPIO_InitStructure);
    }

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                        	// 独立模式
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                     	// 4分频
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;         	// 禁用多ADC模式的DMA访问,不影响ADC1独立模式的DMA请求
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    	// 5个采样延迟
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          	// 12位分辨率
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    	// 启用扫描转换模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                             	// 启用连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;     	// 无外部触发转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;           	// 选择TIM1_CC1作为外部触发源,触发边沿为None时不生效
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          	// 右对齐数据
    ADC_InitStructure.ADC_NbrOfConversion = ADC_CHANNEL_COUNT;                      	// 规则组转换通道数量
    AD_DMAInit();
    ADC_Init(ADC_USED, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC_USED, ADC_IU_CHANNEL, 1, ADC_SampleTime_84Cycles); 	// 配置I_U为规则组第1位,采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_TEMP_CHANNEL, 2, ADC_SampleTime_84Cycles); 	// 配置Temp为规则组第2位,采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_UBUS_CHANNEL, 3, ADC_SampleTime_84Cycles); 	// 配置Ubus为规则组第3位,采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_IW_CHANNEL, 4, ADC_SampleTime_84Cycles); 	// 配置I_W为规则组第4位,采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_POT_CHANNEL, 5, ADC_SampleTime_84Cycles); 	// 配置POT为规则组第5位,采样时间为84个周期

    ADC_DMACmd(ADC_USED, ENABLE);
    ADC_DMARequestAfterLastTransferCmd(ADC_USED, ENABLE);
    ADC_Cmd(ADC_USED, ENABLE);
    Delay_us(10);
    ADC_SoftwareStartConv(ADC_USED);
}

int16_t AD_GetValue(ADC_CHANNEL_ID channel)
{
    if ((uint32_t)channel >= (uint32_t)ADC_CHANNEL_COUNT)
        return -1;

    return (uint16_t)AD_Value[channel];
}
