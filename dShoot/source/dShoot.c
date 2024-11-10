/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-09-26 11:21:23
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-10-03 11:40:19
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431cbu6_BLDC\dShoot\source\dShoot.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "..\include\dShoot.h"
#include <stddef.h>
#include <stdbool.h>

#include "main.h"

uint16_t dmaTransmitBuffer[16 + 2];
uint16_t dmaReceiveBuffer[16 + 2];

uint32_t dShoot_GetTransmitBufferAddress(void)
{
    return (uint32_t)&dmaTransmitBuffer[0];
}

uint32_t dShoot_GetReceiveBufferAddress(void)
{
    return (uint32_t)&dmaReceiveBuffer[0];
}

uint16_t dShoot_CalculateWithCRC(uint16_t value,uint8_t checkBit)
{
    uint16_t pack = (value << 1) | (checkBit ? 1 : 0);
    uint16_t sum = 0,tmp = pack;
    for(int i = 0;i < 3;i++){
        sum ^= tmp;tmp >>= 4;
    }
    pack = (pack << 4) | (sum & 0xf);
    return pack;
}

void dShoot_ValueEncode(uint16_t value,uint8_t checkBit)
{
    value = (value > 2047) ? 2047 : value;
    value = dShoot_CalculateWithCRC(value,checkBit);
    for(int i = 0;i < 16;i++){
        dmaTransmitBuffer[i] = (value & 0x8000) ? dShoot_Logical1_DMA_Value : dShoot_Logical0_DMA_Value;
        value <<= 1;
    }
    dmaTransmitBuffer[16] = 0;
    dmaTransmitBuffer[17] = 0;
}

void dShoot_ValueTransmitWithDMA(uint16_t value,uint8_t checkBit)
{
    dShoot_ValueEncode(value,checkBit);
    dShoot_TransmitValueWithDMA();
}

void dShoot_TransmitValueWithDMA(void)
{
    if(LL_DMA_IsActiveFlag_TC1(DMA1)){
        LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_1);
        LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,16 + 2);
        LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
    }
}

uint16_t dShoot_ReceiveValueWithDMA(void)
{
    uint16_t value = 0x0000;
    uint16_t realData = 0x0000;

    for(int i = 0;i < 16;i++){
        realData <<= 1;
        if(dmaReceiveBuffer[i] > dShoot_Logical1_DMA_Value - 15 && dmaReceiveBuffer[i] < dShoot_Logical1_DMA_Value + 15){
            realData |= 0x1;
        }
        else if(dmaReceiveBuffer[i] > dShoot_Logical0_DMA_Value - 15 && dmaReceiveBuffer[i] < dShoot_Logical0_DMA_Value + 15){
            realData = realData;
        }else{
            return 0x0000;
        }
    }
    uint8_t checkBit = realData & 0x10;
    value = realData >> 5;

    if(dShoot_CalculateWithCRC(value,checkBit) != realData) 
        return 0x0000;

    return value;
}





