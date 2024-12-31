/**
 * @file alg_power_limit.h
 * @author jh,qyx
 * @brief 自适应功率限制算法
 * @version 1.2
 * @date 
 *
 * @copyright ZLLC 2025
 *
 */

#ifndef ALG_POWER_LIMIT_H
#define ALG_POWER_LIMIT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
#include "config.h"
#include "RLS.hpp"

/* Exported macros -----------------------------------------------------------*/
#define REDUATION (3591.f / 187.f) // 标准减速比
#define RAD_TO_RPM 9.5493f
#define CMD_CURRENT_TO_TORQUE_CURRENT (20.f / 16384.f) // Icmd映射到Itorque
#define Kt 0.3 / REDUATION                             // 转子的转矩常数

#define CMD_CURRENT_TO_TORQUE (CMD_CURRENT_TO_TORQUE_CURRENT * Kt)
#define TORQUE_TO_CMD_CURRENT 1 / (CMD_CURRENT_TO_TORQUE_CURRENT * Kt)
#define PI 3.14159265354f

typedef struct
{
    __fp16 feedback_omega;  // 反馈的转子转速,rpm
    __fp16 feedback_torque; // 反馈的转子转矩,Nm

    __fp16 torque;           // pid输出的转子转矩,Nm
    float theoretical_power; // 理论功率
    float scaled_power;      // 功率（缩放后）

    int16_t pid_output;     // pid输出的扭矩电流控制值（16384）
    int16_t output;        // 最终输出扭矩电流控制值（16384）
} Struct_Power_Motor_Data;

typedef struct
{
    uint16_t Max_Power;            // 最大功率限制
    float Scale_Conffient;         // 功率缩放系数
    float Theoretical_Total_Power; // 理论总功率
    float Scaled_Total_Power;      // 缩放后总功率
    float Actual_Power;            // 实际总功率

    Struct_Power_Motor_Data Motor_Data[8]; // 舵轮八个电机，分为四组，默认偶数索引值的电机为转向电机，奇数索引值的电机为动力电机

} Struct_Power_Management;

class Class_Power_Limit
{
public:
    float Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index);
    float Calculate_Toque(float omega, float power, float torque, uint8_t motor_index);
    void Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data);
    void Power_Task(Struct_Power_Management &power_management);
    void Init();

#ifdef AGV
    // AGV模式下的getter/setter
    inline float Get_K1_Mot() const { return k1_mot; }
    inline float Get_K2_Mot() const { return k2_mot; }
    inline float Get_K1_Dir() const { return k1_dir; }
    inline float Get_K2_Dir() const { return k2_dir; }
    inline float Get_K3_Mot() const { return k3_mot; }
    inline float Get_K3_Dir() const { return k3_dir; }

    inline void Set_K1_Mot(float _k1) { k1_mot = _k1; }
    inline void Set_K2_Mot(float _k2) { k2_mot = _k2; }
    inline void Set_K1_Dir(float _k1) { k1_dir = _k1; }
    inline void Set_K2_Dir(float _k2) { k2_dir = _k2; }
    inline void Set_K3_Mot(float _k3) { k3_mot = _k3; }
    inline void Set_K3_Dir(float _k3) { k3_dir = _k3; }
#else
    // 普通模式下的getter/setter
    inline float Get_K1() const { return k1; }
    inline float Get_K2() const { return k2; }
    inline float Get_K3() const { return k3; }

    inline void Set_K1(float _k1) { k1 = _k1; }
    inline void Set_K2(float _k2) { k2 = _k2; }
    inline void Set_K3(float _k3) { k3 = _k3; }
#endif

protected:
#ifdef AGV
    // AGV模式参数
    float k1_mot = 0.024246;   // 动力电机k1
    float k2_mot = 1.183594;   // 动力电机k2
    float k3_mot = 9.28f/8.0f; // 动力电机k3
    
    float k1_dir = 0.024246;   // 转向电机k1
    float k2_dir = 1.183594;   // 转向电机k2
    float k3_dir = 9.28f/8.0f; // 转向电机k3
    
    RLS<2> rls_mot{1e-5f, 0.9999f}; // 动力电机RLS
    RLS<2> rls_dir{1e-5f, 0.9999f}; // 转向电机RLS
#else
    // 普通模式参数
    float k1 = 0.024246;
    float k2 = 1.183594;
    float k3 = 9.28f/8.0f;
    RLS<2> rls{1e-5f, 0.9999f};
#endif
};

#endif