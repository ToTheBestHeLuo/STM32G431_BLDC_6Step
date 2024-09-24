#ifndef _BLDC_TASK_H_
#define _BLDC_TASK_H_

#include "bldcType.h"

extern BLDC_System_Handler bldcSysHandler;
extern BLDC_Sensor_Handler bldcSensorHandler;
extern BLDC_Sensorless_Handler bldcSensorlessHandler;

extern void bldc_SafetyTask(void);
extern void bldc_HighFrequencyTask(void);

// extern void bldc_SetLowSidesStatus(void);
#endif
