/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:50:48
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-24 20:12:58
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\include\bldcSensorless.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _BLDC_SENSORLESS_H_
#define _BLDC_SENSORLESS_H_

#include "../include/bldcType.h"

extern PIController spPIController;

extern uint16_t speedPIController(f32_t spNow);
extern void excuteUserCMD(void);
extern int8_t bldcSensorlessEstSpeed(f32_t floatingPhaseVoltage,f32_t busVoltage);

#endif

