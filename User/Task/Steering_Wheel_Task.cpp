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

// 更新功率控制所需的数据
#ifdef AGV_BOARD_A
    steering_wheel->Power_Management.Motor_Data[0].omega = steering_wheel->Directive_Motor.Get_Now_Omega_Radian() * steering_wheel->Directive_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[0].torque = steering_wheel->Directive_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    steering_wheel->Power_Management.Motor_Data[1].omega = steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * steering_wheel->Motion_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[1].torque = steering_wheel->Motion_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    memcpy(AGV_BOARD_CAN_DATA, &steering_wheel->Power_Management.Motor_Data[0].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 2, &steering_wheel->Power_Management.Motor_Data[0].torque, 2);

    memcpy(AGV_BOARD_CAN_DATA + 4, &steering_wheel->Power_Management.Motor_Data[1].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 6, &steering_wheel->Power_Management.Motor_Data[1].torque, 2);
#endif

#ifdef AGV_BOARD_B
    steering_wheel->Power_Management.Motor_Data[2]
        .omega = steering_wheel->Directive_Motor.Get_Now_Omega_Radian() * steering_wheel->Directive_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[2].torque = steering_wheel->Directive_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    steering_wheel->Power_Management.Motor_Data[3].omega = steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * steering_wheel->Motion_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[3].torque = steering_wheel->Motion_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    memcpy(AGV_BOARD_CAN_DATA, &steering_wheel->Power_Management.Motor_Data[2].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 2, &steering_wheel->Power_Management.Motor_Data[2].torque, 2);

    memcpy(AGV_BOARD_CAN_DATA + 4, &steering_wheel->Power_Management.Motor_Data[3].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 6, &steering_wheel->Power_Management.Motor_Data[3].torque, 2);
#endif

#ifdef AGV_BOARD_C
    steering_wheel->Power_Management.Motor_Data[4].omega = steering_wheel->Directive_Motor.Get_Now_Omega_Radian() * steering_wheel->Directive_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[4].torque = steering_wheel->Directive_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    steering_wheel->Power_Management.Motor_Data[5].omega = steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * steering_wheel->Motion_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[5].torque = steering_wheel->Motion_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    memcpy(AGV_BOARD_CAN_DATA, &steering_wheel->Power_Management.Motor_Data[4].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 2, &steering_wheel->Power_Management.Motor_Data[4].torque, 2);

    memcpy(AGV_BOARD_CAN_DATA + 4, &steering_wheel->Power_Management.Motor_Data[5].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 6, &steering_wheel->Power_Management.Motor_Data[5].torque, 2);
#endif

#ifdef AGV_BOARD_D
    steering_wheel->Power_Management.Motor_Data[6].omega = steering_wheel->Directive_Motor.Get_Now_Omega_Radian() * steering_wheel->Directive_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[6].torque = steering_wheel->Directive_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    steering_wheel->Power_Management.Motor_Data[7].omega = steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * steering_wheel->Motion_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[7].torque = steering_wheel->Motion_Motor.Get_Now_Torque() * CMD_CURRENT_TO_TORQUE;

    memcpy(AGV_BOARD_CAN_DATA, &steering_wheel->Power_Management.Motor_Data[6].omega, 2)
        memcpy(AGV_BOARD_CAN_DATA + 2, &steering_wheel->Power_Management.Motor_Data[6].torque, 2);

    memcpy(AGV_BOARD_CAN_DATA + 4, &steering_wheel->Power_Management.Motor_Data[7].omega, 2);
    memcpy(AGV_BOARD_CAN_DATA + 6, &steering_wheel->Power_Management.Motor_Data[7].torque, 2);
#endif
}

void Command_Update(Class_Steering_Wheel *steering_wheel)
{
    
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
    steering_wheel->Motion_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Velocity / Wheel_Diameter * 2); // Get_Now_Omega_Radian获得的是电机输出轴的角速度，这个角速度放在了data里，Set_Now_Omega_Radian设置的是电机类直属的Now_Omega_Radian

    steering_wheel->Directive_Motor.Set_Target_Radian(steering_wheel->Target_Angle * DEG_TO_RAD);
    steering_wheel->Directive_Motor.Set_Now_Radian(steering_wheel->Now_Angle * DEG_TO_RAD);       // 转向轮的当前数据不直接来自于电机
    steering_wheel->Directive_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Omega * DEG_TO_RAD); // 转向轮的当前数据不直接来自于电机

    // PID计算
    steering_wheel->Motion_Motor.TIM_PID_PeriodElapsedCallback();
    steering_wheel->Directive_Motor.TIM_PID_PeriodElapsedCallback();

#if POWER_CONTROL == 1

    // 运行功率限制任务
    steering_wheel->Power_Limit.Power_Task(&steering_wheel->Power_Management);

    // 更新电机输出值
#ifdef AGV_BOARD_A
    steering_wheel->Directive_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[0].output);
    steering_wheel->Motion_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[1].output);
#endif

#ifdef AGV_BOARD_B
    steering_wheel->Directive_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[2].output);
    steering_wheel->Motion_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[3].output);
#endif

#ifdef AGV_BOARD_C
    steering_wheel->Directive_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[4].output);
    steering_wheel->Motion_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[5].output);
#endif

#ifdef AGV_BOARD_D
    steering_wheel->Directive_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[6].output);
    steering_wheel->Motion_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[7].output);
#endif

#endif // POWER_CONTROL==1
}

void Command_Send(Class_Steering_Wheel *steering_wheel)
{
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);        // 发送本轮组电机指令
    CAN_Send_Data(&hcan2, AGV_BOARD_ID, AGV_BOARD_CAN_DATA, 8); // 发送本轮组电机的转速和扭矩
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