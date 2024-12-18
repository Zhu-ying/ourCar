# PID库实现的笔记
---
*草台班子*

---
#### PID_Control_Inc() 增量式PID控制算法
这段代码是一个增量式PID控制算法的实现。PID控制是一种常见的反馈控制方法，用于工业控制系统中，以保持系统的输出与期望值（设定值）尽可能接近。增量式PID算法与位置式PID算法的主要区别在于，增量式PID算法计算的是控制量的变化量（增量），而不是控制量的绝对值。

下面是这段代码的逐行解释：

1. `float PID_Control_Inc(PID* pid, int flag)`：
   接受两个参数：一个指向`PID`结构体的指针`pid`和一个整数`flag`。

2. `float inc = 0;`：
   这个变量将用来存储计算出的控制量增量。

3. `pid->ek = pid->SetValue - pid->ActualValue;`：
   计算当前的误差`ek`，即设定值`SetValue`与实际值`ActualValue`之间的差值。

4. `inc = pid->KP * (pid->ek - pid->ek_1) + pid->KI * pid->ek + pid->KD * (pid->ek - 2 * pid->ek_1 + pid->ek_2);`：
   这是增量式PID算法的核心计算部分。
   - `pid->KP`是比例系数，`pid->ek - pid->ek_1`是误差的变化量，这部分是比例项。
   - `pid->KI`是积分系数，`pid->ek`是当前误差，这部分是积分项。
   - `pid->KD`是微分系数，`pid->ek - 2 * pid->ek_1 + pid->ek_2`是误差的差分，这部分是微分项。
   - 将这些项相加得到控制量增量`inc`。

5. `pid->ek_2 = pid->ek_1;` 和 `pid->ek_1 = pid->ek;`：
   更新误差的历史值，为下一次计算做准备。`ek_2`变成`ek_1`，`ek_1`变成当前的`ek`。

6. `if(flag == 1)`：
   如果`flag`参数为1，那么执行以下限制操作，以确保控制量增量不会超过设定的最大值或最小值。
   - `if(inc > pid->PIDmax) inc = pid->PIDmax;`：如果`inc`大于最大值`PIDmax`，则将`inc`设置为`PIDmax`。
   - `if(inc < pid->PIDmin) inc = pid->PIDmin;`：如果`inc`小于最小值`PIDmin`，则将`inc`设置为`PIDmin`。

7. `pid->PIDout = inc;`：
   将计算出的控制量增量`inc`赋值给`PIDout`，这是结构体中用来存储输出的地方。

8. `return pid->PIDout;`：返回计算出的控制量增量，即`PIDout`的值。

增量式PID控制算法，用于计算控制量的增量，并根据需要限制这个增量的值。这种算法在需要快速响应和减少积分饱和的场合非常有用。


#### PID_Control_Pos() 位置式PID控制算法
位置式PID算法与增量式PID算法的主要区别在于，位置式PID算法直接计算控制量（PID输出）的绝对值，而不是控制量的变化量（增量）。
下面是这段代码的逐行解释：

1. `float PID_Control_Pos(PID* pid, int flag)`：
   它接受两个参数：一个指向`PID`结构体的指针`pid`和一个整数`flag`。

2. `float Pos = 0;`：
   这个变量将用来存储计算出的控制量。

3. `pid->ek = pid->SetValue - pid->ActualValue;`：
   计算当前的误差`ek`，即设定值`SetValue`与实际值`ActualValue`之间的差值。

4. `pid->ek_sum += pid->ek;`：
   计算误差的累积和`ek_sum`，这是积分项的计算部分。

5. `if(pid->ek_sum > pid->Sum_max) pid->ek_sum = pid->Sum_max;` 和 `if(pid->ek_sum < pid->Sum_min) pid->ek_sum = pid->Sum_min;`：
   对误差累积和进行限制，以防止积分饱和。如果`ek_sum`超过了最大值`Sum_max`或最小值`Sum_min`，则将其限制在这些值内。

6. `Pos = pid->KP * pid->ek + pid->KI * pid->ek_sum + pid->KD * (pid->ek - pid->ek_1);`：
   这是位置式PID算法的核心计算部分。
   - `pid->KP`是比例系数，`pid->ek`是当前误差，这部分是比例项。
   - `pid->KI`是积分系数，`pid->ek_sum`是误差的累积和，这部分是积分项。
   - `pid->KD`是微分系数，`pid->ek - pid->ek_1`是误差的变化量，这部分是微分项。
   - 将这些项相加得到控制量`Pos`。

7. `pid->ek_2 = pid->ek_1;` 和 `pid->ek_1 = pid->ek;`：
   更新误差的历史值，为下一次计算做准备。`ek_2`变成`ek_1`，`ek_1`变成当前的`ek`。

8. `if(flag == 1)`：
   如果`flag`参数为1，那么执行以下限制操作，以确保控制量不会超过设定的最大值或最小值。
   - `if(Pos > pid->PIDmax) Pos = pid->PIDmax;`：如果`Pos`大于最大值`PIDmax`，则将`Pos`设置为`PIDmax`。
   - `if(Pos < pid->PIDmin) Pos = pid->PIDmin;`：如果`Pos`小于最小值`PIDmin`，则将`Pos`设置为`PIDmin`。

9.  `pid->PIDout = Pos;`：
    将计算出的控制量`Pos`赋值给`PIDout`，这是结构体中用来存储输出的地方。

10. `return pid->PIDout;`：
    返回计算出的控制量，即`PIDout`的值。

总的来说，这个函数实现了位置式PID控制算法，用于计算控制量的绝对值，并根据需要限制这个值。这种算法在需要直接控制输出值的场合非常有用，例如在电机控制和其他需要精确控制的系统中。
