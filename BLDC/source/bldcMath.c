#include "../include/bldcMath.h"

f32_t MedianFilter(f32_t datBuffer[],uint16_t length)
{
    f32_t res = 0.f;

    if(length < 3){
        return res;
    }

    for(int i = 0;i < length - 1;++i){
        for(int j = 0;j < length - i - 1;++j){
            if(datBuffer[j] > datBuffer[j + 1]){
                f32_t tmp = datBuffer[j];
                datBuffer[j] = datBuffer[j + 1];
                datBuffer[j + 1] = tmp;
            }
        }
    }

    return datBuffer[length / 2];
}
