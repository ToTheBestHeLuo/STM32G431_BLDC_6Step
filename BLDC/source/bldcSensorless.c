/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:52:21
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-09-25 16:14:52
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\source\bldcSensorless.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/bldcSensorless.h"
#include "../include/bldcInterface.h"
#include "../include/bldcTask.h"

PIController spPIController = {
    .userCmd = eBLDC_CMD_IDLE,
    .finalTarget = 0.f,
    .timeCost = 0.f,
    .kI = 100.f,
    .kP = 4.f,
    .limitMax = 7000.f / 2.f,
    .limitMin = 600.f / 2.f,
    .integrator = 0.f,
    .out = 0.f,
    .target = 0.f
};

void setTargetRotationFrequency(f32_t target) {spPIController.target = target * 2.f * 3.1415926f;}
f32_t getTargetRotationFrequency(void) {return spPIController.target / 2.f / 3.1415926f;}

void setTargetAngularSpeed(f32_t target) {spPIController.target = target;}
f32_t getTargetAngularSpeed(void) {return spPIController.target;}

void setTargetRotationFrequencyWithACC(f32_t target,f32_t timeNeed)
{
    if(timeNeed < 0.0005f) return;

    if(spPIController.userCmd == eBLDC_CMD_IDLE){
        spPIController.timeCost = timeNeed;
        spPIController.finalTarget = target;
        spPIController.acc = (target - getTargetRotationFrequency()) / timeNeed;
        spPIController.userCmd = eBLDC_CMD_SPEED_UPDOWN;
    }
}

void excuteUserCMD(void)
{
    static f32_t time = 0.f;
    switch(spPIController.userCmd){
        case eBLDC_CMD_IDLE:
            break;
        case eBLDC_CMD_SPEED_UPDOWN:
            time += bldcSysHandler.highSpeedClock;
            setTargetRotationFrequency(spPIController.acc * bldcSysHandler.highSpeedClock + getTargetRotationFrequency());
            if(time > spPIController.timeCost){
                time = 0.f;
                spPIController.userCmd = eBLDC_CMD_IDLE;
            }
            break;
        case eBLDC_CMD_DIR_REVERSE:
            bldcSensorlessHandler.forceAlignmentDir = ~bldcSensorlessHandler.forceAlignmentDir;
            bldcSysHandler.sysStu = eWaitSysReset;
            spPIController.userCmd = eBLDC_CMD_IDLE;
            break;
        default:
            break;
    }
}

uint16_t speedPIController(f32_t spNow)
{
    f32_t err = spPIController.target - spNow;

    f32_t out = err * spPIController.kP + spPIController.integrator * spPIController.kI;

    spPIController.integrator += err * bldcSysHandler.highSpeedClock;

    if(spPIController.integrator < -20.f){
        spPIController.integrator = -20.f;
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

int8_t bldcSensorlessEstSpeed(f32_t floatingPhaseVoltage,f32_t busVoltage,int8_t* zeroCrossCnt)
{
    if(floatingPhaseVoltage > busVoltage / 2.f){
        *zeroCrossCnt = *zeroCrossCnt + 1;
        if(*zeroCrossCnt > 3){
            *zeroCrossCnt = 3;
            return 1;
        }
    }else{
        *zeroCrossCnt = *zeroCrossCnt - 1;
        if(*zeroCrossCnt < -3){
            *zeroCrossCnt = -3;
            return -1;
        }
    }
    return 0;
}

