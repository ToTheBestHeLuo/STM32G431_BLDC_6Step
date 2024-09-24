/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:11:33
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-23 20:54:25
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\include\bldcInterface.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _BLDC_INTERFACE_H_
#define _BLDC_INTERFACE_H_

#include "bldcType.h"

extern f32_t (*bldcSwitchPhaseTableList_Forward[6])(uint16_t);
extern f32_t (*bldcSwitchPhaseTableList_Reverse[6])(uint16_t);

extern void bldc_LowFrequencyTaskCallBack(void);
extern void bldc_HighFrequencyTaskCallBack(void);
extern uint8_t bldc_StartOrStopMotor(void);

// extern void bldc_PWM_LowSides_OFF_ON_OFF(void);
// extern void bldc_PWM_LowSides_OFF_OFF_ON(void);
// extern void bldc_PWM_LowSides_ON_OFF_OFF(void);

// extern f32_t bldc_BEMF_Get_U(void);
// extern f32_t bldc_BEMF_Get_V(void);
// extern f32_t bldc_BEMF_Get_W(void);

extern void bldc_StartPWM_AllChannel_HighSides(void);
extern void bldc_StopPWM_AllChannel_HighSides(void);
extern void bldc_StartPWM_AllChannel_LowSides(void);
extern void bldc_StopPWM_AllChannel_LowSides(void);
extern f32_t bldc_GetBusVoltage(void);
extern f32_t bldc_GetBusCurrentAndSetNextSamplingPosition(uint16_t samplingPositionCnt);

#endif

