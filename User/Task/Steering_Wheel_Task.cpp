#include "Steering_Wheel_Task.h"
#include "FreeRTOS.h"

// 应该要加互斥锁
void State_Update(Class_Steering_Wheel *steering_wheel)
{
    // 由于齿轮传动使得编码器转动方向为CW时，舵转动方向为CCW，反之亦然。所以要对称处理
    /*
        编码器顺时针旋转：     0° -> 90° -> 180°
        舵轮实际逆时针旋转：  360° -> 270° -> 180°

        所以需要：新角度 = 360° - 编码器角度
    */
    steering_wheel->Now_Angle = 360.0 - steering_wheel->Encoder.Data.Now_Angle * ENCODER_TO_OUTPUT_RATIO;
    steering_wheel->Now_Omega = -steering_wheel->Encoder.Data.Now_Omega * ENCODER_TO_OUTPUT_RATIO;
    steering_wheel->Now_Velocity = steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * Wheel_Diameter / 2; // 这里有点怪异，因为Get_Now_Omega_Radian和Set_Target_Omega_Radian返回的数据不一样
}

void Command_Update(Class_Steering_Wheel *steering_wheel)
{
    // todo
}

/*
初始状态：
Now_Angle = 0°
Target_Angle = 135°
invert_flag = 0（不反转）

1. 角度优化：
   temp_err = 135° - 0° - (0 * 180°) = 135°

   比较路径：
   - 直接路径 = |135°| = 135°
   - 绕行路径 = 360° - |135°| = 225°
   temp_min = min(135°, 225°) = 135°

   因为 temp_min(135°) > 90°：
   - invert_flag 切换为 1（反转）
   - 重新计算误差：
     temp_err = 135° - 0° - (1 * 180°) = -45°

2. 优劣弧优化：
   temp_err = -45° 已经在[-180°, 180°]范围内，无需调整

3. 最终结果：
   - 电机反转
   - 逆时针转动45°
*/
void Control_Update(Class_Steering_Wheel *steering_wheel)
{
    float temp_err = 0.0f;

    // 1. 角度优化
    if (steering_wheel->parameter.deg_optimization == ENABLE_MINOR_DEG_OPTIMIZEATION)
    {
        float temp_min;

        // 计算误差，考虑当前电机状态
        temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;

        // 标准化到[0, 360)范围
        while (temp_err > 360.0f)
            temp_err -= 360.0f;
        while (temp_err < 0.0f)
            temp_err += 360.0f;

        // 比较路径长度
        if (fabs(temp_err) < (360.0f - fabs(temp_err)))
            temp_min = fabs(temp_err);
        else
            temp_min = 360.0f - fabs(temp_err);

        // 判断是否需要切换方向
        if (temp_min > 90.0f)
        {
            steering_wheel->invert_flag = !steering_wheel->invert_flag;
            // 重新计算误差
            temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;
        }
    }
    else
    {
        temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle;
    }

    // 2. 优劣弧优化，实际上角度优化那里已经完成了
    if (steering_wheel->parameter.arc_optimization == ENABLE_MINOR_ARC_OPTIMIZEATION)
    {
        if (temp_err > 180.0f)
        {
            temp_err -= 360.0f;
        }
        else if (temp_err < -180.0f)
        {
            temp_err += 360.0f;
        }
    }

    steering_wheel->Target_Angle = steering_wheel->Now_Angle + temp_err;

    steering_wheel->Motion_Motor.Set_Target_Omega_Radian(steering_wheel->Target_Velocity / Wheel_Diameter * 2 * steering_wheel->invert_flag);
    steering_wheel->Motion_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Velocity / Wheel_Diameter * 2); // 这里有点怪异，因为Get_Now_Omega_Radian和Set_Target_Omega_Radian返回的数据不一样

    steering_wheel->Directive_Motor.Set_Target_Radian(steering_wheel->Target_Angle * DEG_TO_RAD);
    steering_wheel->Directive_Motor.Set_Now_Radian(steering_wheel->Now_Angle * DEG_TO_RAD);//转向轮的当前数据不直接来自于电机
    steering_wheel->Directive_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Omega * DEG_TO_RAD);//转向轮的当前数据不直接来自于电机


    //PID控制
    steering_wheel->Motion_Motor.TIM_PID_PeriodElapsedCallback();
    steering_wheel->Directive_Motor.TIM_PID_PeriodElapsedCallback();
}

void Steering_Wheel_Task(void const *pvParameters)
{
    static float steering_wheel_dt;
    static float steering_wheel_start;
    while (1)
    {
        steering_wheel_start = DWT_GetTimeline_ms();

        State_Update(&steering_wheel);
        Command_Update(&steering_wheel);
        Control_Update(&steering_wheel);

        steering_wheel_dt = DWT_GetTimeline_ms() - steering_wheel_start;
        osDelayUntil(1);
    }
}