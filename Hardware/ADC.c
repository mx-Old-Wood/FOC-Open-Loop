#include "stm32f4xx.h" // Device header
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
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;         	// 禁用DMA访问
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    	// 5个采样延迟
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          	// 12位分辨率
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    	// 启用扫描转换模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             	// 禁用连续转换模式    
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;     	// 无外部触发转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;           	// 外部触发转换源为TIM1_CC1
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          	// 右对齐数据
    ADC_InitStructure.ADC_NbrOfConversion = ADC_CHANNEL_COUNT;                      	// 转换通道数量
    ADC_Init(ADC_USED, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC_USED, ADC_IU_CHANNEL, 1, ADC_SampleTime_84Cycles); 	// 配置通道I_U_CHANNEL为单通道转换，采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_TEMP_CHANNEL, 2, ADC_SampleTime_84Cycles); 	// 配置通道TEMP_CHANNEL为单通道转换，采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_UBUS_CHANNEL, 3, ADC_SampleTime_84Cycles); 	// 配置通道UBUS_CHANNEL为单通道转换，采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_IW_CHANNEL, 4, ADC_SampleTime_84Cycles); 	// 配置通道IW_CHANNEL为单通道转换，采样时间为84个周期
    ADC_RegularChannelConfig(ADC_USED, ADC_POT_CHANNEL, 5, ADC_SampleTime_84Cycles); 	// 配置通道POT_CHANNEL为单通道转换，采样时间为84个周期
}
