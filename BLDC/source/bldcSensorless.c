/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:52:21
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-11-06 21:23:17
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\source\bldcSensorless.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/bldcSensorless.h"
#include "../include/bldcInterface.h"
#include "../include/bldcTask.h"

PIController spPIController = {
    .kI = 100.f * 0.15f,
    .kP = 10.f * 0.15f,
    .limitMax = 8500 - 200,
    .limitMin = 500,
    .integrator = 0.f,
    .out = 0.f,
    .target = 0.f
};

void excuteUserCMD(void)
{
	if(spPIController.target < spPIController.finalTarget * 0.95f){
        spPIController.target += 0.00005f * 2.f * 3.1415926f * 1000.f;
    }else if(spPIController.target > spPIController.finalTarget * 1.05f){
        spPIController.target -= 0.00005f * 2.f * 3.1415926f * 1000.f;
    }else{
        spPIController.target = spPIController.finalTarget;
    }
}

uint16_t speedPIController(f32_t spNow)
{
    f32_t err = spPIController.target - spNow;

    f32_t out = err * spPIController.kP + spPIController.integrator * spPIController.kI;

    spPIController.integrator += err * bldcSysHandler.lowSpeedClock;

    if(spPIController.integrator < -2000.f){
        spPIController.integrator = -2000.f;
    }else if(spPIController.integrator > 2000.f){
        spPIController.integrator = 2000.f;
    }

    if(out > spPIController.limitMax){
        out = spPIController.limitMax;
    }
    else if(out < spPIController.limitMin){
         out = spPIController.limitMin;
    }

    spPIController.out = out;

    return (uint16_t)out;
}

int8_t bldcSensorlessEstSpeed(f32_t floatingPhaseVoltage,f32_t busVoltage)
{
    if(floatingPhaseVoltage > busVoltage / 2.f * 1.05f) return 1;
    else if(floatingPhaseVoltage < busVoltage / 2.f * 0.95f) return -1;
    return 0;
}

