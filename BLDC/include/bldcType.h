/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-24 15:08:50
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-09-25 15:43:37
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\BLDC\include\bldcType.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _BLDC_TYPE_H_
#define _BLDC_TYPE_H_

typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

typedef float f32_t;
typedef double f64_t;

typedef enum{
    eBLDC_CMD_IDLE = 0,
    eBLDC_CMD_SPEED_UPDOWN,
    eBLDC_CMD_DIR_REVERSE
}BLDC_CMD_Status;

typedef struct 
{
    BLDC_CMD_Status userCmd;
    f32_t finalTarget,acc,timeCost;
    f32_t limitMax,limitMin;
    f32_t kP,kI,integrator;
    f32_t out,target;
}PIController;

typedef struct{
    uint16_t pwmDuty;
}BLDC_PWM_Handler;

typedef struct{
    f32_t busVoltage,busCurrent,driverTemp;
    f32_t floatingPhaseX_Voltage;
}BLDC_Sensor_Handler;

typedef struct{
    int8_t switchPhaseSignal;
    int8_t zeroCrossCnt,zeroCrossSignal;
    uint8_t forceAlignmentSector,forceAlignmentDir;
    uint16_t forceAlignmentPwmDuty,forceAlignmentTime;
    f32_t estSpeed;
}BLDC_Sensorless_Handler;

typedef enum{
    eBLDC_UOFF_VON_WOFF = 0,
    eBLDC_UOFF_VOFF_WON,
    eBLDC_UON_VOFF_WOFF
}BLDC_PWM_LowSidesStatus;

typedef enum{
    eBLDC_SAMPLING_BEMF_U = 0,
    eBLDC_SAMPLING_BEMF_V,
    eBLDC_SAMPLING_BEMF_W,
    eBLDC_SAMPLING_BEMF_RESET
}BLDC_BEMF_SamplingStatus;

typedef enum{
    eWaitSysReset = -1,
    eWaitBusVoltage,
    eWaitCapCharge,
    eWaitMCStart,
    eSysRun
}BLDC_SysStateMachine;

typedef enum{
    eBLDC_ERROR_NONE = 0,
    eBLDC_ERROR_OC,
    eBLDC_ERROR_OV,
    eBLDC_ERROR_UV,
    eBLDC_ERROR_OT
}BLDC_SysErrorStatus;

typedef enum{
    eBLDC_ForceAlignment = 0,
    eBLDC_OpenLoop,
    eBLDC_CloseLoopRun,
}BLDC_RunStateMachine;

typedef struct 
{
    uint64_t sysRunTimeCnt;
    uint32_t safetyTaskTimeCnt;
    uint32_t highSpeedTimeCnt;
    f32_t lowSpeedClock,highSpeedClock,safeBusVoltage;

    BLDC_PWM_LowSidesStatus lowSidesStatus;
    BLDC_BEMF_SamplingStatus bemfSamplingStatus;

    BLDC_SysStateMachine sysStu;
    BLDC_SysErrorStatus sysError;
    BLDC_RunStateMachine bldcStu;
}BLDC_System_Handler;

#endif

