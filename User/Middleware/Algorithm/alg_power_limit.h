/**
 * @file alg_power_limit.h
 * @author lez
 * @brief 功率限制算法
 * @version 1.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef ALG_POWER_LIMIT_H
#define ALG_POWER_LIMIT_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
#include "config.h"

/* Exported macros -----------------------------------------------------------*/
#define REDUATION (3591.f / 187.f) // 标准减速比
#define RAD_TO_RPM 9.5493f
#define CMD_CURRENT_TO_TORQUE_CURRENT (20.f / 16384.f) // Icmd映射到Itorque
#define Kt 0.3 / REDUATION                             // 转子的转矩常数

#define CMD_CURRENT_TO_TORQUE (CMD_CURRENT_TO_TORQUE_CURRENT * Kt)
#define TORQUE_TO_CMD_CURRENT 1 / (CMD_CURRENT_TO_TORQUE_CURRENT * Kt)

typedef struct
{
    float theoretical; // 理论功率
    float scaled;      // 功率（缩放后）
} Motor_Power_t;

typedef struct
{
    Motor_Power_t motion;    // 动力电机功率
    Motor_Power_t directive; // 转向电机功率
} Steering_Wheel_Power_t;    // 舵轮功率结构体

//RLS参数更新需要所有电机的数据
typedef struct
{
    float omega;  // 转子转速,rpm
    float torque; // 转子转矩,Nm
} Motor_Data_t;

typedef struct
{
    Motor_Data_t motion;
    Motor_Data_t directive;
} Steering_Wheel_Motor_Data_t;

typedef struct
{
    uint16_t Max_Power;            // 最大功率限制
    float Scale_Conffient;         // 功率缩放系数
    float Theoretical_Total_Power; // 理论总功率
    float Scaled_Total_Power;      // 缩放后总功率

    Steering_Wheel_Power_t Steering_Wheel_Power[4];
    Steering_Wheel_Motor_Data_t Steering_Wheel_Motor_Data[4];

} Struct_Power_Management; // 功率管理结构体

class Class_Power_Limit
{
public:
    float Calculate_Theoretical_Power(float omega, float torque); // 计算单个电机的理论功率
    float Calculate_Toque(float omega, float power);              // 根据功率计算转矩

protected:
    // 转矩系数 rad转rpm系数
    float Toque_Coefficient = 1.99688994e-6f * (3591 / 187) / 13.93f; // (20/16384)*(0.3)*(187/3591)/9.55

    // 电机模型参数
    float k1 = 1.3;   // k1
    float k2 = 0.015; // k2
    float k3=8.4/8.0;// k3 静态损耗/n
    float Alpha = 0.0f;
    float Tansfer_Coefficient = 9.55f; // 转化系数 w*t/Tansfer_Coefficient
};

/* Exported types ------------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
