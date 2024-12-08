/**
 * @file alg_power_limit.cpp
 * @author lez
 * @brief 功率限制算法
 * @version 1.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_power_limit.h"
#include "math.h"
#include "RLS.hpp"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_Power_Limit Power_Limit;
RLS rls;
/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }

static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f; }

static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float fmax(float a, float b) {   return (a > b) ? a : b;}
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 返回单个电机的计算功率
 *
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @return float
 */
float Class_Power_Limit::Calculate_Theoretical_Power(float omega, float torque)
{
    float cmdPower;

    float sumError = 0.0f;
    float error;

    float k1 = Get_K1();
    float k2 = Get_K2();

    cmdPower = omega * torque + fabs(omega) * k1 + torque * torque * k2 +
               k3 ;
    return cmdPower;
}

float Class_Power_Limit::Calculate_Sum_Power(Struct_Power_Management &power_management)
{
    float sumPower = 0;

    for (int i = 0; i < 4; i++)
    {
        power_management.Theoretical_Total_Power += Calculate_Theoretical_Power(power_management.Steering_Wheel_Motor_Data[i].directive.omega,
                                                                                power_management.Steering_Wheel_Motor_Data[i].directive.torque) +
                                                    Calculate_Theoretical_Power(power_management.Steering_Wheel_Motor_Data[i].motion.omega,
                                                                                power_management.Steering_Wheel_Motor_Data[i].motion.torque);
    }
    sumPower = power_management.Theoretical_Total_Power;
    return sumPower;
}

float Class_Power_Limit::Calculate_Scaled_Coefficient(float max_power, float theoretical_sum_power, Struct_Power_Management &power_management)
{
    float scaled_coefficient = 0;
    theoretical_sum_power = Calculate_Sum_Power(power_management);
    scaled_coefficient = max_power / theoretical_sum_power;
    power_management.Scale_Conffient = scaled_coefficient;
    return scaled_coefficient;
}

float Class_Power_Limit::Calculate_Power_Coefficient(Struct_Power_Management &power_management)
{

    // todo
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    static float effectivePower = 0;
    // We consider motor that is just disconnected as still using power, by assuming the motor keep
    // latest output and rpm by 1 second, otherwise it is not safe if we only use energy loop
    for (int i = 0; i < 4; i++)
    {
        if (power_management.Actual_Power > 5)
        {
            effectivePower += power_management.Steering_Wheel_Motor_Data[i].directive.torque *
                                  rpm2av(power_management.Steering_Wheel_Motor_Data[i].directive.omega) +
                              power_management.Steering_Wheel_Motor_Data[i].motion.torque * rpm2av(power_management.Steering_Wheel_Motor_Data[i].motion.omega);

            samples[0][0] += fabsf(rpm2av(power_management.Steering_Wheel_Motor_Data[i].motion.omega)) +
                             fabsf(rpm2av(power_management.Steering_Wheel_Motor_Data[i].directive.omega));

            samples[1][0] += power_management.Steering_Wheel_Motor_Data[i].motion.torque * power_management.Steering_Wheel_Motor_Data[i].motion.torque + power_management.Steering_Wheel_Motor_Data[i].directive.torque * power_management.Steering_Wheel_Motor_Data[i].directive.torque;
        }
    }
    // power_limit.Motor_Power.theoretical = power_limit.k1 * samples[0][0] + power_limit.k2 * samples[1][0] + effectivePower + power_limit.k3;
    params = rls.update(samples, power_management.Actual_Power - effectivePower -8*k3);
    k1 = fmax(params[0][0], 1e-5f); // In case the k1 diverge to negative number
    k2 = fmax(params[1][0], 1e-5f); // In case the k2 diverge to negative number
}
/**
 * @brief 返回扭矩，单位为Nm，用返回值的时候要转化到控制量
 *
 * @param omega 转子转速，单位为rpm
 * @param power 电机功率，单位为w
 * @return float
 */
float Class_Power_Limit::Calculate_Toque(float omega, float power, Struct_Power_Management &power_management, float torque)
{
    float newTorqueCurrent = 0.0f;
    float powerWeight = power_management.Scale_Conffient;
    float delta = omega*omega-4*(k1*fabs(omega)+k3-powerWeight*power)*k2;
    if (floatEqual(delta, 0.0f)) // repeat roots
    {
        newTorqueCurrent = omega / (2.0f * k2);
    }
    else if (delta > 0.0f) // distinct roots
    {
        newTorqueCurrent = torque > 0.0f ? (omega + sqrtf(delta)) / (2.0f * k2)
                                         : (omega - sqrtf(delta)) / (2.0f * k2);
    }
    else // imaginary roots
    { 
        newTorqueCurrent = omega / (2.0f * k2);
    }
   // newTorqueCurrent = Utils::Math::clamp(newTorqueCurrent, p->pidMaxOutput);


return newTorqueCurrent;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
