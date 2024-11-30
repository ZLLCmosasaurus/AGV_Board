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

#define RAD_TO_RPM              9.5493f
#define CMD_CURRENT_TO_TORQUE   ((20.f/16384.f)*0.3f)   //计算出3508直驱输出轴转子和cmd电流的关系
#define REDUATION               (3591.f/187.f)         //减速比

typedef struct
{
    float theoretical; // 理论功率
    float scaled;        // 功率（缩放后）
} Motor_Power_t;

typedef struct
{
    Motor_Power_t motion;   // 动力电机功率
    Motor_Power_t directive; // 转向电机功率
} Steering_Wheel_Power_t;   // 舵轮功率结构体

class Class_Power_Limit
{
    public:
        void Calculate_Theoretical_Power();
        Strcut_Steering_Weel_Power Steering_Weel_Power[4];

    protected:
      


        // 转矩系数 rad转rpm系数
        float Toque_Coefficient = 1.99688994e-6f * (3591 / 187) / 13.93f; // (20/16384)*(0.3)*(187/3591)/9.55

        // 电机模型参数
        float k1 = 1.3;   // k1
        float k2 = 0.015; // k2
        float Alpha = 0.0f;
        float Tansfer_Coefficient = 9.55f; // 转化系数 w*t/Tansfer_Coefficient

};


/* Exported types ------------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
