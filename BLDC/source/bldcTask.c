/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 16:20:16
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-09-25 16:10:14
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\source\bldcTask.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/bldcTask.h"
#include "../include/bldcInterface.h"
#include "../include/bldcMath.h"
#include "../include/bldcSensorless.h"

BLDC_System_Handler bldcSysHandler = {
    .sysRunTimeCnt = 0,
    .highSpeedClock = 1.f / 40.f / 1000.f,
    .lowSpeedClock = 0.001f,
    .safeBusVoltage = 36.f,
    .highSpeedTimeCnt = 0u,
    .safetyTaskTimeCnt = 0u,
    .lowSidesStatus = eBLDC_UOFF_VOFF_WON,
    .bemfSamplingStatus = eBLDC_SAMPLING_BEMF_RESET,
    .sysStu = eWaitSysReset,
    .sysError = eBLDC_ERROR_NONE,
    .bldcStu = eBLDC_ForceAlignment
};

BLDC_Sensor_Handler bldcSensorHandler = {
    .busCurrent = 0.f,
    .busVoltage = 0.f,
    .driverTemp = 0.f,
    .floatingPhaseX_Voltage = 0.f
};

BLDC_Sensorless_Handler bldcSensorlessHandler = {
    .zeroCrossSignal = 0,
    .zeroCrossCnt = 0,
    .switchPhaseSignal = 1,
    .forceAlignmentSector = 0u,
    .forceAlignmentDir = 0xff,
    .forceAlignmentPwmDuty = 200,
    .forceAlignmentTime = 199,
    .estSpeed = 0.f
};
uint8_t bldc_DetermineButtonStatus(void)
{
    static uint8_t lastButtonStatus = 0x00;
    int8_t buttonStatus = bldc_GetButtonStatus();
    if(buttonStatus == 1)
        lastButtonStatus = ~lastButtonStatus;
    return lastButtonStatus;
}

void bldc_SysReset(void)
{
    bldcSysHandler.bldcStu = eBLDC_ForceAlignment;
    bldcSensorlessHandler.forceAlignmentSector = 0;
    bldcSensorlessHandler.forceAlignmentPwmDuty = 200;
    spPIController.target = 0.f;
    spPIController.integrator = 0.f;
    spPIController.finalTarget = 0.f;
    spPIController.timeCost = 0.f;
    spPIController.userCmd = eBLDC_CMD_IDLE;
}

void bldc_SafetyTask(void)
{
    static f32_t busFilter[10] = {0.f};
    const uint32_t lengthBusFilter = sizeof(busFilter) / sizeof(f32_t);
    switch(bldcSysHandler.sysStu){
        case eWaitSysReset:
            bldc_StopPWM_AllChannel_HighSides();
            bldc_SysReset();
            if(bldcSysHandler.safetyTaskTimeCnt++ > 500u){
                bldcSysHandler.safetyTaskTimeCnt = 0; 
                bldcSysHandler.sysStu = eWaitBusVoltage;         
            }
            break;
        case eWaitBusVoltage:
            busFilter[bldcSysHandler.sysRunTimeCnt % lengthBusFilter] = bldc_GetBusVoltage();
            bldcSensorHandler.busVoltage = MedianFilter(busFilter,lengthBusFilter);
            if(bldcSensorHandler.busVoltage > bldcSysHandler.safeBusVoltage * 0.9f && bldcSensorHandler.busVoltage < bldcSysHandler.safeBusVoltage * 1.1f){
                if(bldcSysHandler.safetyTaskTimeCnt++ > 50u){
                    bldcSysHandler.safetyTaskTimeCnt = 0;
                    bldcSysHandler.sysStu = eWaitCapCharge;            
                }
            }
            else bldcSysHandler.safetyTaskTimeCnt = 0;
            break;
        case eWaitCapCharge:
            if(bldcSysHandler.safetyTaskTimeCnt++ > 50u){
                bldcSysHandler.highSpeedTimeCnt = 0;
                bldcSysHandler.safetyTaskTimeCnt = 0;
                bldcSysHandler.sysStu = eWaitMCStart;
            }
            else if(bldcSysHandler.safetyTaskTimeCnt > 45u) bldc_StartPWM_AllChannel_HighSides();
            break;
        case eWaitMCStart:
            if(bldc_DetermineButtonStatus()){
                bldcSysHandler.sysStu = eSysRun;
            }
            break;
        default:
            break;
    }
    if(bldcSysHandler.sysStu == eSysRun){
        if(!bldc_DetermineButtonStatus()) bldcSysHandler.sysStu = eWaitSysReset;
        busFilter[bldcSysHandler.sysRunTimeCnt % (sizeof(busFilter) / sizeof(f32_t))] = bldc_GetBusVoltage();
        bldcSensorHandler.busVoltage = MedianFilter(busFilter,(sizeof(busFilter) / sizeof(f32_t)));
        if(bldcSensorHandler.busVoltage < bldcSysHandler.safeBusVoltage * 0.8f){
            if(bldcSysHandler.safetyTaskTimeCnt++ > 100u){
                bldc_StopPWM_AllChannel_HighSides();
                bldcSysHandler.safetyTaskTimeCnt = 0;
                bldcSysHandler.sysError = eBLDC_ERROR_UV;
                bldcSysHandler.sysStu = eWaitSysReset;
            }
        }
        else if(bldcSensorHandler.busVoltage > bldcSysHandler.safeBusVoltage * 1.4f){
            if(bldcSysHandler.safetyTaskTimeCnt++ > 100u){
                bldc_StopPWM_AllChannel_HighSides();
                bldcSysHandler.safetyTaskTimeCnt = 0;
                bldcSysHandler.sysError = eBLDC_ERROR_OV;
                bldcSysHandler.sysStu = eWaitSysReset;
            }
        }
        else bldcSysHandler.safetyTaskTimeCnt = 0;
    }
    bldcSensorHandler.driverTemp = bldc_GetDriverTemp();

    if(bldcSensorHandler.driverTemp > 65.f){
        bldc_StopPWM_AllChannel_HighSides();
        bldcSysHandler.sysError = eBLDC_ERROR_OT;
        bldcSysHandler.sysStu = eWaitSysReset;
    }

    bldcSysHandler.sysRunTimeCnt++;
    bldc_LowFrequencyTaskCallBack();
}

void bldc_HighFrequencyTask(void)
{
    f32_t estSpeedFilter = 0.f;
    int8_t switchPhase;
    static uint32_t tmp = 0u;
    static f32_t timeCost = 0.f;
    bldcSensorHandler.busCurrent = bldc_GetBusCurrentAndSetNextSamplingPosition(bldcSensorlessHandler.forceAlignmentPwmDuty / 2);
    if(bldcSysHandler.sysStu == eSysRun){ 
        switch (bldcSysHandler.bldcStu)
        {
            case eBLDC_ForceAlignment:
                bldcSwitchPhaseTableList_Forward[0](bldcSensorlessHandler.forceAlignmentPwmDuty);
                if(bldcSysHandler.highSpeedTimeCnt++ == 1000){
                    tmp = 0u;estSpeedFilter = 0.f;timeCost = 0.f;
                    bldcSysHandler.highSpeedTimeCnt = 0u;
                    bldcSysHandler.bldcStu = eBLDC_OpenLoop;
                }
                break;
            case eBLDC_OpenLoop:
                if(bldcSensorlessHandler.forceAlignmentDir){
                    bldcSensorHandler.floatingPhaseX_Voltage = bldcSwitchPhaseTableList_Forward[bldcSensorlessHandler.forceAlignmentSector](bldcSensorlessHandler.forceAlignmentPwmDuty);
                    // bldcSensorHandler.floatingPhaseX_Voltage = bldc_GetBEMF();
                }else{
                    bldcSensorHandler.floatingPhaseX_Voltage = bldcSwitchPhaseTableList_Reverse[bldcSensorlessHandler.forceAlignmentSector](bldcSensorlessHandler.forceAlignmentPwmDuty);
                    // bldcSensorHandler.floatingPhaseX_Voltage = bldc_GetBEMF();
                }
                if(bldcSysHandler.highSpeedTimeCnt++ == bldcSensorlessHandler.forceAlignmentTime){
                    bldcSysHandler.highSpeedTimeCnt = 0u;
                    bldcSensorlessHandler.forceAlignmentSector = (bldcSensorlessHandler.forceAlignmentSector + 1) % 6;
                }
                timeCost += bldcSysHandler.highSpeedClock;
                switchPhase = bldcSensorlessEstSpeed(bldcSensorHandler.floatingPhaseX_Voltage,bldcSensorHandler.busVoltage,&bldcSensorlessHandler.zeroCrossCnt);
                if(bldcSensorlessHandler.switchPhaseSignal == -switchPhase){
                    bldcSensorlessHandler.switchPhaseSignal = -bldcSensorlessHandler.switchPhaseSignal;
                    estSpeedFilter = 3.1415926f / 3.f / timeCost;
                    timeCost = 0.f;
                    bldcSensorlessHandler.estSpeed = 0.05f * estSpeedFilter + bldcSensorlessHandler.estSpeed * 0.95f;
                    if(++tmp == 50){
                        tmp = 0u;
                        setTargetAngularSpeed(bldcSensorlessHandler.estSpeed);
                        bldcSensorlessHandler.forceAlignmentSector = (bldcSensorlessHandler.forceAlignmentSector + 1) % 6;
                        bldcSysHandler.bldcStu = eBLDC_CloseLoopRun;
                    }
                }
                break;
            case eBLDC_CloseLoopRun:
                if(bldcSensorlessHandler.forceAlignmentDir){
                    bldcSensorHandler.floatingPhaseX_Voltage = bldcSwitchPhaseTableList_Forward[bldcSensorlessHandler.forceAlignmentSector](bldcSensorlessHandler.forceAlignmentPwmDuty);
                    // bldcSensorHandler.floatingPhaseX_Voltage = bldc_GetBEMF();
                }else{
                    bldcSensorHandler.floatingPhaseX_Voltage = bldcSwitchPhaseTableList_Reverse[bldcSensorlessHandler.forceAlignmentSector](bldcSensorlessHandler.forceAlignmentPwmDuty);
                    // bldcSensorHandler.floatingPhaseX_Voltage = bldc_GetBEMF();
                }
                timeCost += bldcSysHandler.highSpeedClock;
                switchPhase = bldcSensorlessEstSpeed(bldcSensorHandler.floatingPhaseX_Voltage,bldcSensorHandler.busVoltage,&bldcSensorlessHandler.zeroCrossCnt);
                if(bldcSensorlessHandler.switchPhaseSignal == -switchPhase){
                    bldcSensorlessHandler.switchPhaseSignal = -bldcSensorlessHandler.switchPhaseSignal;
                    estSpeedFilter = 3.1415926f / 3.f / timeCost;
                    timeCost = 0.f;       
                    bldcSensorlessHandler.estSpeed = 0.05f * estSpeedFilter + bldcSensorlessHandler.estSpeed * 0.95f;
                    bldcSensorlessHandler.forceAlignmentSector = (bldcSensorlessHandler.forceAlignmentSector + 1) % 6;
                }
                bldcSensorlessHandler.forceAlignmentPwmDuty = speedPIController(bldcSensorlessHandler.estSpeed);
                excuteUserCMD();
                break;
            default:
                break;
        }
    }
    bldc_HighFrequencyTaskCallBack();
}

