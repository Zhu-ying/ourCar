#include "headfile.h"

float PID_Control_Inc(PID* pid, int flag) // 增量式PID
{
    float inc = 0;

    pid->ek = pid->SetValue - pid->ActualValue;

    inc = pid->KP * (pid->ek - pid->ek_1) + pid->KI * pid->ek
          + pid->KD * (pid->ek - 2 * pid->ek_1 + pid->ek_2);

    pid->ek_2 = pid->ek_1; //存储误差
    pid->ek_1 = pid->ek; //存储误差

    if(flag == 1)
    {
        if(inc > pid->PIDmax)
            inc = pid->PIDmax;
        if(inc < pid->PIDmin)
            inc = pid->PIDmin;
    }
    pid->PIDout = inc;
    return pid->PIDout;
}

float PID_Control_Pos(PID* pid, int flag) // 位置式PID
{
    float Pos = 0;

    pid->ek = pid->SetValue - pid->ActualValue;
    pid->ek_sum += pid->ek;

    if(pid->ek_sum > pid->Sum_max)
        pid->ek_sum = pid->Sum_max;
    if(pid->ek_sum < pid->Sum_min)
        pid->ek_sum = pid->Sum_min;

    Pos = pid->KP * pid->ek + pid->KI * pid->ek_sum + pid->KD * (pid->ek - pid->ek_1);

    pid->ek_2 = pid->ek_1; //存储误差
    pid->ek_1 = pid->ek; //存储误差

    // 积分限幅
    if(flag == 1)
    {
        if(Pos > pid->PIDmax)
            Pos = pid->PIDmax;
        if(Pos < pid->PIDmin)
            Pos = pid->PIDmin;
    }
    pid->PIDout = Pos;
	
    return pid->PIDout;
}

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
