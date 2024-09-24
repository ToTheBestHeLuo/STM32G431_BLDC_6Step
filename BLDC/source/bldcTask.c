/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 16:20:16
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-24 20:30:34
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\source\bldcTask.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/bldcTask.h"
#include "../include/bldcInterface.h"
#include "../include/bldcMath.h"
#include "../include/bldcSensorless.h"

static f32_t busCurrentOffsetFilter[100] = {0.f};
const uint32_t lengthCurrentFilter = sizeof(busCurrentOffsetFilter) / sizeof(f32_t);

BLDC_System_Handler bldcSysHandler = {
    .sysRunTimeCnt = 0,
    .highSpeedClock = 1.f / 20.f / 1000.f,
    .lowSpeedClock = 0.001f,
    .safeBusVoltage = 36.f,
    .highSpeedTimeCnt = 0u,
    .isBusCurrenfOffsetFinished = 0u,
    .safetyTaskTimeCnt = 0u,
    .lowSidesStatus = eBLDC_UOFF_VOFF_WON,
    .bemfSamplingStatus = eBLDC_SAMPLING_BEMF_RESET,
    .sysStu = eWaitSysReset,
    .bldcStu = eBLDC_ForceAlignment
};

BLDC_Sensor_Handler bldcSensorHandler = {
    .busCurrent = 0.f,
    .busVoltage = 0.f,
    .driverTemp = 0.f,
    .floatingPhaseX_Voltage = 0.f,
    .samplingPosition = 340u
};

BLDC_Sensorless_Handler bldcSensorlessHandler = {
    .zeroCrossSignal = 0,
    .zeroCrossCnt = 0,
    .switchPhaseSignal = 1,
    .forceAlignmentSector = 0u,
    .forceAlignmentDir = 1u,
    .forceAlignmentPwmDuty = 499,
    .forceAlignmentTime = 199,
    .maxPwmDuty = 8400,
    .minPwmDuty = 272 * 2,
    .estSpeed = 0.f
};

// f32_t bldc_GetBEMF(void)
// {
//     switch(bldcSysHandler.bemfSamplingStatus){
//         case eBLDC_SAMPLING_BEMF_U:
//             return bldc_BEMF_Get_U();
//         case eBLDC_SAMPLING_BEMF_V:
//             return bldc_BEMF_Get_V();
//         case eBLDC_SAMPLING_BEMF_W:
//             return bldc_BEMF_Get_W();
//         default:
//             return 0.f;
//     }
// }

// void bldc_SetLowSidesStatus(void)
// {
//     switch(bldcSysHandler.lowSidesStatus){
//         case eBLDC_UOFF_VON_WOFF:
//             bldc_PWM_LowSides_OFF_ON_OFF();
//             break;
//         case eBLDC_UOFF_VOFF_WON:
//             bldc_PWM_LowSides_OFF_OFF_ON();
//             break;
//         default:
//             bldc_PWM_LowSides_ON_OFF_OFF();
//             break;
//     }
// }

void bldc_SafetyTask(void)
{
    static f32_t busFilter[10] = {0.f};
    const uint32_t lengthBusFilter = sizeof(busFilter) / sizeof(f32_t);
    switch(bldcSysHandler.sysStu){
        case eWaitSysReset:
            bldc_StopPWM_AllChannel_HighSides();
            bldcSysHandler.sysStu = eWaitBusVoltage;
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
                bldcSysHandler.sysStu = eWaitCalADCOffset;
            }
            else if(bldcSysHandler.safetyTaskTimeCnt > 45u) bldc_StartPWM_AllChannel_HighSides();
            break;
        case eWaitCalADCOffset:
            if(bldcSysHandler.isBusCurrenfOffsetFinished){
                bldcSensorHandler.busCurrentOffset = MedianFilter(busCurrentOffsetFilter,lengthCurrentFilter);
                bldcSysHandler.safetyTaskTimeCnt = 0;
                bldcSysHandler.sysStu = eWaitMCStart;
            }
            break;
        case eWaitMCStart:
            if(bldc_StartOrStopMotor()){
                bldcSysHandler.sysStu = eSysRun;
            }
            break;
        default:
            break;
    }
    if(bldcSysHandler.sysStu == eSysRun){
        if(!bldc_StartOrStopMotor()) bldcSysHandler.sysStu = eWaitSysReset;
        busFilter[bldcSysHandler.sysRunTimeCnt % (sizeof(busFilter) / sizeof(f32_t))] = bldc_GetBusVoltage();
        bldcSensorHandler.busVoltage = MedianFilter(busFilter,(sizeof(busFilter) / sizeof(f32_t)));
        if(bldcSensorHandler.busVoltage < bldcSysHandler.safeBusVoltage * 0.85f){
            if(bldcSysHandler.safetyTaskTimeCnt++ > 140u){
                bldc_StopPWM_AllChannel_HighSides();
                bldcSysHandler.safetyTaskTimeCnt = 0;
                bldcSysHandler.sysStu = eWaitSysReset;
            }
        }
        else if(bldcSensorHandler.busVoltage > bldcSysHandler.safeBusVoltage * 1.4f){
            if(bldcSysHandler.safetyTaskTimeCnt++ > 140u){
                bldc_StopPWM_AllChannel_HighSides();
                bldcSysHandler.safetyTaskTimeCnt = 0;
                bldcSysHandler.sysStu = eWaitSysReset;
            }
        }
        else bldcSysHandler.safetyTaskTimeCnt = 0;
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
        bldcSensorHandler.busCurrent -= bldcSensorHandler.busCurrentOffset;
        switch (bldcSysHandler.bldcStu)
        {
            case eBLDC_ForceAlignment:
                bldcSwitchPhaseTableList_Forward[0](bldcSensorlessHandler.forceAlignmentPwmDuty);
                if(bldcSysHandler.highSpeedTimeCnt++ == 6000){
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
                    bldcSensorlessHandler.switchPhaseTime = timeCost / 7.f;
                    timeCost = 0.f;
                    bldcSensorlessHandler.estSpeed = 0.05f * estSpeedFilter + bldcSensorlessHandler.estSpeed * 0.95f;
                    if(++tmp == 50){
                        tmp = 0u;
                        setTargetAngularSpeed(bldcSensorlessHandler.estSpeed);
                        bldcSysHandler.bldcStu = eBLDC_SwitchPhase;
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
                    bldcSensorlessHandler.switchPhaseTime = timeCost / 7.f;
                    timeCost = 0.f;       
                    bldcSensorlessHandler.estSpeed = 0.05f * estSpeedFilter + bldcSensorlessHandler.estSpeed * 0.95f;
                    bldcSysHandler.bldcStu = eBLDC_SwitchPhase;
                }
                bldcSensorlessHandler.forceAlignmentPwmDuty = speedPIController(bldcSensorlessHandler.estSpeed);
                excuteUserCMD();
                break;
            case eBLDC_SwitchPhase:
                timeCost += bldcSysHandler.highSpeedClock;
                bldcSysHandler.highSpeedTimeCnt++;
                if((f32_t)bldcSysHandler.highSpeedTimeCnt * bldcSysHandler.highSpeedClock > bldcSensorlessHandler.switchPhaseTime){
                    bldcSysHandler.highSpeedTimeCnt = 0;
                    bldcSensorlessHandler.forceAlignmentSector = (bldcSensorlessHandler.forceAlignmentSector + 1) % 6;
                    bldcSysHandler.bldcStu = eBLDC_CloseLoopRun;
                }
                break;
            default:
                break;
        }
    }else if(bldcSysHandler.sysStu == eWaitCalADCOffset && !bldcSysHandler.isBusCurrenfOffsetFinished){
        if(bldcSysHandler.highSpeedTimeCnt++ != lengthCurrentFilter){
            busCurrentOffsetFilter[bldcSysHandler.highSpeedTimeCnt] = bldcSensorHandler.busCurrent;
        }else{
            bldcSysHandler.highSpeedTimeCnt = 0;
            bldcSysHandler.isBusCurrenfOffsetFinished = 1u;
        }
    }
    bldc_HighFrequencyTaskCallBack();
}

