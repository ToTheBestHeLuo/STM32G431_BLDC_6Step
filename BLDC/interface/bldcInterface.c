/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:11:02
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-09-25 15:59:04
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\interface\bldcInterface.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/bldcInterface.h"
#include "../include/bldcTask.h"
#include "../include/bldcSensorless.h"
#include "usbd_cdc_if.h"
#include "main.h"

void bldc_LowFrequencyTaskCallBack(void)
{
    if((bldcSysHandler.sysRunTimeCnt % 1000) == 0){
        LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_14);
        LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_15);
    }
    static uint16_t index = 0u;
    uint16_t index1 = index % sizeof(UserRxBufferFS);
    uint16_t index2 = (index + 1) % sizeof(UserRxBufferFS);
    uint16_t index3 = (index + 2) % sizeof(UserRxBufferFS);
    uint16_t index4 = (index + 3) % sizeof(UserRxBufferFS);
    if(UserRxBufferFS[index1] == 'S' && UserRxBufferFS[index2] == ':' && UserRxBufferFS[index4] == '\n'){
        uint16_t receiveDat = UserRxBufferFS[index3] * 6.f;
        setTargetRotationFrequencyWithACC(receiveDat,6.f);
        UserRxBufferFS[index1] = UserRxBufferFS[index2] = '\0';
        UserRxBufferFS[index3] = UserRxBufferFS[index4] = '\0';
    }
    index++;
}

void bldc_HighFrequencyTaskCallBack(void)
{
    struct bufToSend
    {
      float dat0,dat1,dat2,dat3,dat4,dat5;
      uint8_t tail[4];
    }buf = {
        bldcSensorlessHandler.estSpeed,
        bldcSensorlessHandler.forceAlignmentSector,
        bldcSensorHandler.busCurrent,
        bldcSensorHandler.driverTemp,
        bldcSensorHandler.busVoltage / 2.f,
        bldcSensorHandler.floatingPhaseX_Voltage,
        0x00,0x00,0x80,0x7f};
    CDC_Transmit_FS((uint8_t*)&buf,sizeof(buf));
}

static uint16_t keyCnt = 0u;

int8_t bldc_GetButtonStatus(void)
{
    uint32_t keySignal = LL_GPIO_ReadInputPort(GPIOA) & (LL_GPIO_PIN_3);
    if(keySignal == 0u && keyCnt++ > 300){
        keyCnt = 0;
        return 1;
    }else if(keySignal) 
        keyCnt = 0;
    return 0;
}

void bldc_StartPWM_AllChannel_HighSides(void)
{
    LL_TIM_OC_SetCompareCH1(TIM1,0);
    LL_TIM_OC_SetCompareCH2(TIM1,0);
    LL_TIM_OC_SetCompareCH3(TIM1,0);
    LL_TIM_OC_SetCompareCH4(TIM1,1);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM1);
}

void bldc_StopPWM_AllChannel_HighSides(void)
{
    LL_TIM_OC_SetCompareCH1(TIM1,0);
    LL_TIM_OC_SetCompareCH2(TIM1,0);
    LL_TIM_OC_SetCompareCH3(TIM1,0);
    LL_TIM_OC_SetCompareCH4(TIM1,0);
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH4);
    LL_TIM_DisableCounter(TIM1);
}

void bldc_StartPWM_AllChannel_LowSides(void)
{
    LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_13);
    LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_0);
    LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_15);
}

void bldc_StopPWM_AllChannel_LowSides(void)
{
    LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_13);
    LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_0);
    LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_15);  
}

void bldc_PWM_LowSides_OFF_ON_OFF(void)
{
    LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_13);
    LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_0);
    LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_15);
}

void bldc_PWM_LowSides_OFF_OFF_ON(void)
{
    LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_13);
    LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_0);
    LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_15);
}

void bldc_PWM_LowSides_ON_OFF_OFF(void)
{
    LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_13);
    LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_0);
    LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_15);
}

f32_t bldc_BEMF_Get_U(void)
{
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
    return (f32_t)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 4096.f * 3.3f * 401.f;
}

f32_t bldc_BEMF_Get_V(void)
{
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_5);
    return (f32_t)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 4096.f * 3.3f * 401.f;
}

f32_t bldc_BEMF_Get_W(void)
{
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_14);
    return (f32_t)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 4096.f * 3.3f * 401.f;
}

f32_t bldc_StartPWM_UH_VL_GetWF(uint16_t pwmDuty)
{
    LL_TIM_OC_SetCompareCH1(TIM1,pwmDuty);
    LL_TIM_OC_SetCompareCH2(TIM1,0);
    LL_TIM_OC_SetCompareCH3(TIM1,0);
    // bldcSysHandler.lowSidesStatus = eBLDC_UOFF_VON_WOFF;
    // bldcSysHandler.bemfSamplingStatus = eBLDC_SAMPLING_BEMF_W;

    bldc_PWM_LowSides_OFF_ON_OFF();
    return bldc_BEMF_Get_W();
}

f32_t bldc_StartPWM_UH_WL_GetVF(uint16_t pwmDuty)
{
    LL_TIM_OC_SetCompareCH1(TIM1,pwmDuty);
    LL_TIM_OC_SetCompareCH2(TIM1,0);
    LL_TIM_OC_SetCompareCH3(TIM1,0);
    // bldcSysHandler.lowSidesStatus = eBLDC_UOFF_VOFF_WON;
    // bldcSysHandler.bemfSamplingStatus = eBLDC_SAMPLING_BEMF_V;
    bldc_PWM_LowSides_OFF_OFF_ON();
    return bldc_BEMF_Get_V();
}

f32_t bldc_StartPWM_VH_WL_GetUF(uint16_t pwmDuty)
{
    LL_TIM_OC_SetCompareCH1(TIM1,0);
    LL_TIM_OC_SetCompareCH2(TIM1,pwmDuty);
    LL_TIM_OC_SetCompareCH3(TIM1,0);
    // bldcSysHandler.lowSidesStatus = eBLDC_UOFF_VOFF_WON;
    // bldcSysHandler.bemfSamplingStatus = eBLDC_SAMPLING_BEMF_U;
    bldc_PWM_LowSides_OFF_OFF_ON();
    return bldc_BEMF_Get_U();
}

f32_t bldc_StartPWM_VH_UL_GetWF(uint16_t pwmDuty)
{
    LL_TIM_OC_SetCompareCH1(TIM1,0);
    LL_TIM_OC_SetCompareCH2(TIM1,pwmDuty);
    LL_TIM_OC_SetCompareCH3(TIM1,0);
    // bldcSysHandler.lowSidesStatus = eBLDC_UON_VOFF_WOFF;
    // bldcSysHandler.bemfSamplingStatus = eBLDC_SAMPLING_BEMF_W;
    bldc_PWM_LowSides_ON_OFF_OFF();
    return bldc_BEMF_Get_W();
}

f32_t bldc_StartPWM_WH_UL_GetVF(uint16_t pwmDuty)
{
    LL_TIM_OC_SetCompareCH1(TIM1,0);
    LL_TIM_OC_SetCompareCH2(TIM1,0);
    LL_TIM_OC_SetCompareCH3(TIM1,pwmDuty);
    // bldcSysHandler.lowSidesStatus = eBLDC_UON_VOFF_WOFF;
    // bldcSysHandler.bemfSamplingStatus = eBLDC_SAMPLING_BEMF_V;
    bldc_PWM_LowSides_ON_OFF_OFF();
    return bldc_BEMF_Get_V();
}

f32_t bldc_StartPWM_WH_VL_GetUF(uint16_t pwmDuty)
{
    LL_TIM_OC_SetCompareCH1(TIM1,0);
    LL_TIM_OC_SetCompareCH2(TIM1,0);
    LL_TIM_OC_SetCompareCH3(TIM1,pwmDuty);
    // bldcSysHandler.lowSidesStatus = eBLDC_UOFF_VON_WOFF;
    // bldcSysHandler.bemfSamplingStatus = eBLDC_SAMPLING_BEMF_U;
    bldc_PWM_LowSides_OFF_ON_OFF();
    return bldc_BEMF_Get_U();
}

f32_t bldc_GetBusVoltage(void)
{
    f32_t busVolatge;
    LL_ADC_REG_StartConversion(ADC1);
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    busVolatge = (float)LL_ADC_REG_ReadConversionData12(ADC1) / 4096.f * 3.3f * 32.f;
    return busVolatge;
}

f32_t bldc_GetDriverTemp(void)
{
    /*温度表：电阻值的步进为10K/100=0.1K，共100个离散数值点*/
    const f32_t thermalTable[] = {\
        25.00000f,25.22635f,25.45535f,25.68705f,25.92152f,26.15882f,26.39902f,26.64217f,26.88836f,27.13765f,\
        27.39011f,27.64583f,27.90488f,28.16734f,28.43330f,28.70286f,28.97609f,29.25310f,29.53399f,29.81885f,\
        30.10780f,30.40095f,30.69841f,31.00030f,31.30675f,31.61789f,31.93386f,32.25479f,32.58085f,32.91217f,\
        33.24893f,33.59129f,33.93943f,34.29354f,34.65381f,35.02045f,35.39366f,35.77368f,36.16074f,36.55509f,\
        36.95699f,37.36672f,37.78456f,38.21082f,38.64583f,39.08993f,39.54348f,40.00687f,40.48050f,40.96480f,\
        41.46023f,41.96729f,42.48649f,43.01840f,43.56359f,44.12271f,44.69645f,45.28553f,45.89073f,46.51291f,\
        47.15297f,47.81191f,48.49078f,49.19076f,49.91310f,50.65917f,51.43047f,52.22865f,53.05552f,53.91305f,\
        54.80345f,55.72916f,56.69287f,57.69760f,58.74676f,59.84414f,60.99406f,62.20141f,63.47180f,64.81163f,\
        66.22836f,67.73064f,69.32863f,71.03435f,72.86217f,74.82945f,76.95741f,79.27231f,81.80723f,84.60453f,\
        87.71967f,91.22705f,95.22956f,99.87492f,105.38544f,112.11690f,120.68861f,132.31892f,149.92791f,183.85694f};

    static f32_t tmp = 0.f;

    LL_ADC_REG_StartConversion(ADC2);
    while(LL_ADC_IsActiveFlag_EOC(ADC2) == 0u);
    f32_t adcDat = (float)LL_ADC_REG_ReadConversionData12(ADC2);
    f32_t realRt = (2.f * adcDat) / (4096.f - adcDat);
    int8_t tableIndex = (10.f - realRt) / 0.1f;
    if(tableIndex < 0) tableIndex = 0;
    else if(tableIndex >= sizeof(thermalTable) / 4) tableIndex = sizeof(thermalTable) / 4 - 2;

    /*欧拉前向插值*/
    f32_t rTemp1 = 10.f - (f32_t)tableIndex * 0.1f;
    f32_t rTemp2 = (realRt - rTemp1) * 10.f * (thermalTable[tableIndex + 1] - thermalTable[tableIndex]) + thermalTable[tableIndex];

    /*低通滤波*/
    tmp = 0.05f * rTemp2 + 0.95f * tmp;

    return tmp;
}

f32_t bldc_GetBusCurrentAndSetNextSamplingPosition(uint16_t samplingPositionCnt)
{   
    f32_t busCurrent = (-1.65f + (float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_1) / 4096.f * 3.3f) / 0.005f / 10.f;
    LL_TIM_OC_SetCompareCH4(TIM1,samplingPositionCnt);
    return busCurrent;
}

f32_t (*bldcSwitchPhaseTableList_Forward[])(uint16_t) = {
    bldc_StartPWM_UH_VL_GetWF,
    bldc_StartPWM_UH_WL_GetVF,
    bldc_StartPWM_VH_WL_GetUF,
    bldc_StartPWM_VH_UL_GetWF,
    bldc_StartPWM_WH_UL_GetVF,
    bldc_StartPWM_WH_VL_GetUF
 };
f32_t (*bldcSwitchPhaseTableList_Reverse[])(uint16_t) = {
    bldc_StartPWM_UH_VL_GetWF,
    bldc_StartPWM_WH_VL_GetUF,
    bldc_StartPWM_WH_UL_GetVF,
    bldc_StartPWM_VH_UL_GetWF,
    bldc_StartPWM_VH_WL_GetUF,
    bldc_StartPWM_UH_WL_GetVF
};

