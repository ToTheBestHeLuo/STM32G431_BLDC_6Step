/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:11:02
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-24 20:11:15
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
        uint16_t receiveDat = UserRxBufferFS[index3];
        setTargetRotationFrequencyWithACC(receiveDat,10.f);
        UserRxBufferFS[index1] = UserRxBufferFS[index2] = '\0';
        UserRxBufferFS[index3] = UserRxBufferFS[index4] = '\0';
    }
    index++;
}

void bldc_HighFrequencyTaskCallBack(void)
{
    struct bufToSend
    {
      float dat0,dat1,dat2,dat3,dat4;
      uint8_t tail[4];
    }buf = {
        bldcSensorlessHandler.estSpeed,
        bldcSensorlessHandler.forceAlignmentSector,
        bldcSensorlessHandler.switchPhaseSignal,
        bldcSensorHandler.busVoltage / 2.f,
        bldcSensorHandler.floatingPhaseX_Voltage,
        0x00,0x00,0x80,0x7f};
    CDC_Transmit_FS((uint8_t*)&buf,sizeof(buf));
}

uint8_t bldc_StartOrStopMotor(void)
{
    static uint8_t isStart = 0u;
    static uint16_t key3Cnt = 0u;
    uint32_t key3Signal = LL_GPIO_ReadInputPort(GPIOA) & (LL_GPIO_PIN_3);
    if(key3Signal) key3Cnt = 0u;
    else{
      if(++key3Cnt > 300u){
        key3Cnt = 0u;
        isStart = 1u;
      }
    }
    return isStart;
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
    return (f32_t)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 4096.f * 3.3f * 401.f;
}

f32_t bldc_BEMF_Get_V(void)
{
    return (f32_t)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_2) / 4096.f * 3.3f * 401.f;
}

f32_t bldc_BEMF_Get_W(void)
{
    return (f32_t)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_1) / 4096.f * 3.3f * 401.f;
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

f32_t bldc_GetBusCurrentAndSetNextSamplingPosition(uint16_t samplingPositionCnt)
{   
    f32_t busCurrent = (-1.65f + (float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_2) / 4096.f * 3.3f) / 0.005f / 20.f;
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
