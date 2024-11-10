/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-09-26 19:47:22
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-09-26 20:40:19
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\dShot\include\dShot.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _DSHOOT_H_
#define _DSHOOT_H_

#include <stdint.h>

#define dShoot_Logical1_DMA_Value (210u * 1u)
#define dShoot_Logical0_DMA_Value (105u * 1u)

extern uint32_t dShoot_GetTransmitBufferAddress(void);
extern uint32_t dShoot_GetReceiveBufferAddress(void);

extern void dShoot_TransmitValueWithDMA(void);
extern uint16_t dShoot_ReceiveValueWithDMA(void);
extern void dShoot_ValueEncode(uint16_t value,uint8_t checkBit);
extern void dShoot_ValueTransmitWithDMA(uint16_t value,uint8_t checkBit);
#endif
