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

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_Power_Limit Power_Limit;

/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }

static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f; }

static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float my_fmax(float a, float b) { return (a > b) ? a : b; }
/* Function prototypes -------------------------------------------------------*/
void Class_Power_Limit::Init()
{
    float initParams[2] = {k1, k2};
    rls.setParamVector(Matrixf<2, 1>(initParams));
}

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

    cmdPower = rpm2av(omega) * torque + fabs(rpm2av(omega)) * k1 + torque * torque * k2 +
               k3;
    return cmdPower;
}

float effectivePower = 0;
float test_total_power = 0;
float k1_part = 0;
float k2_part = 0;
void Class_Power_Limit::Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data)
{
    // 功率计算和RLS更新
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    effectivePower = 0;
    test_total_power = 0;
    // 清零samples
    samples[0][0] = 0;
    samples[1][0] = 0;
    if (actual_power > 5)
    {
        // 遍历所有电机（8个）
        for (int i = 0; i < 8; i++)
        {

            // 计算有效功率
            if (motor_data[i].feedback_torque *
                    rpm2av(motor_data[i].feedback_omega) <=0)
            {
                effectivePower += 0;
            }
            else
            {
                effectivePower += motor_data[i].feedback_torque *
                                  rpm2av(motor_data[i].feedback_omega);
            }
            // 更新样本数据
            samples[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
            samples[1][0] += motor_data[i].feedback_torque *
                             motor_data[i].feedback_torque;
            k1_part = k1 * samples[0][0];
            k2_part = k2 * samples[1][0];
            test_total_power += (effectivePower + k1 * samples[0][0] + k2 * samples[1][0] + k3);
        }

        // RLS更新
        params = rls.update(samples, actual_power - effectivePower - 8 * k3);
        k1 = params[0][0];
        k2 = params[1][0];
        //	  k1=0;
        //	  k2=0;
        // 参数限制
                k1 = my_fmax(params[0][0], 1e-5f); // 防止k1为负
                k2 = my_fmax(params[1][0], 1e-5f); // 防止k2为负
    }
}
/**
 * @brief 返回扭矩，单位为Nm，用返回值的时候要转化到控制量
 *
 * @param omega 转子转速，单位为rpm
 * @param power 电机功率，单位为w
 * @param torque pid输出的扭矩，单位为Nm
 * @return float
 */
float newTorqueCurrent = 0.0f;
float Class_Power_Limit::Calculate_Toque(float omega, float power, float torque)
{
	omega=rpm2av(omega);
    newTorqueCurrent = 0.0f;
    float delta = omega * omega - 4 * (k1 * fabs(omega) + k3 - power) * k2;
	if(power<=0){
		newTorqueCurrent=torque;
	}
	else{
    if (floatEqual(delta, 0.0f)) // repeat roots
    {
        //newTorqueCurrent = omega / (2.0f * k2);
			 newTorqueCurrent = 0;
    }
    else if (delta > 0.0f) // distinct roots
    {
        float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2);
        float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2);

        if (torque > 0.0f)
        {
            newTorqueCurrent = (solution1 > 0.0f) ? solution1 : solution2;
        }
        else
        {
            newTorqueCurrent = (solution1 < 0.0f) ? solution1 : solution2;
        }
    }
    else // imaginary roots
    {
        //newTorqueCurrent = omega / (2.0f * k2);
			 newTorqueCurrent = 0;
    }
    // newTorqueCurrent = Utils::Math::clamp(newTorqueCurrent, p->pidMaxOutput);
	}
    return newTorqueCurrent;
}

/**
 * @brief
 *
 * @param power_management
 */
float test_cur = 0;
float test_five=0;
void Class_Power_Limit::Power_Task(Struct_Power_Management &power_management)
{
    float theoretical_sum = 0;
    float scaled_sum = 0;
    // 计算理论功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].theoretical_power = Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega, power_management.Motor_Data[i].torque);
        if (power_management.Motor_Data[i].theoretical_power <= 0)
            continue;
        theoretical_sum += power_management.Motor_Data[i].theoretical_power;
    }
    power_management.Theoretical_Total_Power = theoretical_sum;

    // 总预期功率大于限制功率时进行收缩
    if (power_management.Max_Power < power_management.Theoretical_Total_Power)
    {
        power_management.Scale_Conffient = power_management.Max_Power / power_management.Theoretical_Total_Power;
    }
    else
    {
        power_management.Scale_Conffient = 1.0f;
    }

    // 更新缩放后的功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].scaled_power = power_management.Motor_Data[i].theoretical_power * power_management.Scale_Conffient;
        scaled_sum += power_management.Motor_Data[i].scaled_power;
    }
    power_management.Scaled_Total_Power = scaled_sum;
    // 更新输出扭矩
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].output = Calculate_Toque(power_management.Motor_Data[i].feedback_omega, power_management.Motor_Data[i].scaled_power, power_management.Motor_Data[i].torque) * TORQUE_TO_CMD_CURRENT;
    if( power_management.Motor_Data[i].output>=16384)
		{
			 power_management.Motor_Data[i].output=0;
		}
		    if( power_management.Motor_Data[i].output<=-16384)
		{
			 power_management.Motor_Data[i].output=0;
		}
			
		}
		power_management.Motor_Data[5].output = Calculate_Toque(power_management.Motor_Data[5].feedback_omega, power_management.Motor_Data[5].scaled_power, power_management.Motor_Data[5].torque) * TORQUE_TO_CMD_CURRENT;

		test_five=power_management.Motor_Data[5].output;
    test_cur = power_management.Motor_Data[5].torque*TORQUE_TO_CMD_CURRENT;
    Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
}

// float Class_Power_Limit::Calculate_Sum_Power(Struct_Power_Management &power_management)
// {
//     for (int i = 0; i < 8; i++)
//     {
//         power_management.Theoretical_Total_Power += power_management.Motor_Data[i].theoretical_power;
//     }

//     return power_management.Theoretical_Total_Power;
// }

// void Class_Power_Limit::Calculate_Scaled_Power(Struct_Power_Management &power_management)
// {
//     power_management.Scaled_Total_Power = power_management.Scale_Conffient * power_management.Theoretical_Total_Power;
//     for (uint8_t i = 0; i < 8; i++)
//     {
//         power_management.Motor_Data[i].scaled_power = power_management.Motor_Data[i].theoretical_power * power_management.Scale_Conffient;
//     }
// }

// float Class_Power_Limit::Calculate_Scaled_Coefficient(Struct_Power_Management &power_management)
// {
//     power_management->Scale_Conffient = power_management.Max_Power / power_management.Theoretical_Total_Power;
//     return power_management->Scale_Conffient;
// }
