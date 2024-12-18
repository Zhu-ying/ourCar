#ifndef _PID_H_
#define _PID_H_

#include "headfile.h"

float PID_Control_Inc(PID* pid, int flag);
float PID_Control_Pos(PID* pid, int flag);

typedef struct // PID结构体定义
{
    float32 SetValue; // 期望值 参考值
    float32 ActualValue; // 实际值
    float32 KP; // 比例因子
    float32 KI; // 积分因子
    float32 KD; // 微分因子
    float32 ek; // 本级误差
    float32 ek_1; // 上一次
    float32 ek_2; // 上上次
    float32 ek_sum; // 误差累积

    float32 Sum_max; // 误差累和上限
    float32 Sum_min; // 误差累和下限
    float32 PIDmax; // max limit
    float32 PIDmin; // min limit

    float32 PIDout; // output
}PID;


#endif _PID_H_