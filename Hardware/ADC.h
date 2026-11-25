#ifndef __ADC_H__
#define __ADC_H__

#define ADC_USED            ADC1                        // ADC使用的ADC
#define ADC_USED_RCC        RCC_APB2Periph_ADC1         // ADC使用的ADC时钟
#define ADC_RESOLUTION      ADC_Resolution_12b          // ADC使用的分辨率位数
#define ADC_IU_CHANNEL      ADC_Channel_15              // I_U通道为15
#define ADC_TEMP_CHANNEL    ADC_Channel_14              // 温度通道为14
#define ADC_UBUS_CHANNEL    ADC_Channel_10              // 母线电压通道为10
#define ADC_IW_CHANNEL      ADC_Channel_9               // W相电流通道为9
#define ADC_POT_CHANNEL     ADC_Channel_4               // 电位器通道为4

typedef enum
{
    I_U = 0,                                            // U相电流
    Temp,                                               // 温度
    Ubus,                                               // 母线电压
    I_W,                                                // W相电流
    POT,                                                // 电位器
    ADC_CHANNEL_COUNT                                   // 通道总数
} ADC_CHANNEL_ID;

void AD_Init(void);
int16_t AD_GetValue(ADC_CHANNEL_ID channel);

#endif
