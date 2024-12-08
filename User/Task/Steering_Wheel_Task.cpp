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

    // PID参数更新
    steering_wheel->Motion_Motor.Set_Target_Omega_Radian(steering_wheel->Target_Velocity / Wheel_Diameter * 2 * steering_wheel->invert_flag);
    steering_wheel->Motion_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Velocity / Wheel_Diameter * 2); // 这里有点怪异，因为Get_Now_Omega_Radian和Set_Target_Omega_Radian返回的数据不一样，准确来说Get_Now_Omega_Radian应该改为Get_Now_Motor_Data_Omega_Radian,但是懒得改了

    steering_wheel->Directive_Motor.Set_Target_Radian(steering_wheel->Target_Angle * DEG_TO_RAD);
    steering_wheel->Directive_Motor.Set_Now_Radian(steering_wheel->Now_Angle * DEG_TO_RAD);       // 转向轮的当前数据不直接来自于电机
    steering_wheel->Directive_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Omega * DEG_TO_RAD); // 转向轮的当前数据不直接来自于电机

    // PID计算
    steering_wheel->Motion_Motor.TIM_PID_PeriodElapsedCallback();
    steering_wheel->Directive_Motor.TIM_PID_PeriodElapsedCallback();

    // todo
    // 以下代码后续需要分离，整个函数功能太长了

    // 更新理论功率
#ifdef AGV_BOARD_A
    steering_wheel->Power_Management.Steering_Wheel_Power[0].motion.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Motion_Motor);
    steering_wheel->Power_Management.Steering_Wheel_Power[0].directive.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Directive_Motor);
#endif //

#ifdef AGV_BOARD_B
    steering_wheel->Power_Management.Steering_Wheel_Power[1].motion.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Motion_Motor);
    steering_wheel->Power_Management.Steering_Wheel_Power[1].directive.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Directive_Motor);
#endif //

#ifdef AGV_BOARD_C
    steering_wheel->Power_Management.Steering_Wheel_Power[2].motion.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Motion_Motor);
    steering_wheel->Power_Management.Steering_Wheel_Power[2].directive.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Directive_Motor);
#endif //

#ifdef AGV_BOARD_D
    steering_wheel->Power_Management.Steering_Wheel_Power[3].motion.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Motion_Motor);
    steering_wheel->Power_Management.Steering_Wheel_Power[3].directive.theoretical = steering_wheel->Power_Limit.Calculate_Theoretical_Power(Directive_Motor);
#endif //

    // 暂时未考虑转向/动力优先级
    //
    for (uint8_t i = 0; i < 4; i++)
    {
        steering_wheel->Power_Management.Theoretical_Total_Power += steering_wheel->Power_Management.Steering_Wheel_Power[i].motion.theoretical + steering_wheel->Power_Management.Steering_Wheel_Power[i].directive.theoretical;
    }

    // todo
    // 这里只是简单的对功率进行缩放，据港科大开源应该还有更优解，待完善
    if (steering_wheel->Power_Management.Theoretical_Total_Power > steering_wheel->Power_Management.Max_Power)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            steering_wheel->Power_Management.Scale_Conffient = steering_wheel->Power_Management.Max_Power / steering_wheel->Power_Management.Theoretical_Total_Power; // 缩放系数等于限制总功率/理论总功率

            steering_wheel->Power_Management.Steering_Wheel_Power[i]
                .motion.scaled = steering_wheel->Power_Management.Steering_Wheel_Power[i].motion.theoretical * steering_wheel->Power_Management.Scale_Conffient;
            steering_wheel->Power_Management.Steering_Wheel_Power[i].directive.scaled = steering_wheel->Power_Management.Steering_Wheel_Power[i].directive.theoretical * steering_wheel->Power_Management.Scale_Conffient;
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            steering_wheel->Power_Management.Steering_Wheel_Power[i].motion.scaled = steering_wheel->Power_Management.Steering_Wheel_Power[i].motion.theoretical;
            steering_wheel->Power_Management.Steering_Wheel_Power[i].directive.scaled = steering_wheel->Power_Management.Steering_Wheel_Power[i].directive.theoretical;
        }
    }


    //根据缩放功率更新输出
    float tmp_torque = 0.0f;
    tmp_torque = steering_wheel->Power_Limit.Calculate_Toque(steering_wheel->Motion_Motor);
    steering_wheel->Motion_Motor.Set_Out(tmp_torque*TORQUE_TO_CMD_CURRENT);
    tmp_torque = steering_wheel->Power_Limit.Calculate_Toque(steering_wheel->Directive_Motor);
    steering_wheel->Directive_Motor.Set_Out(tmp_torque*TORQUE_TO_CMD_CURRENT);

}

void Command_Send(Class_Steering_Wheel *steering_wheel)
{
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data,8);     //发送本轮组电机指令
    CAN_Send_Data(&hcan2, AGV_BOARD_ID, AGV_BOARD_CAN_DATA, 8); //发送本轮组电机的转速和扭矩
}

void Steering_Wheel_Task(void *pvParameters)
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