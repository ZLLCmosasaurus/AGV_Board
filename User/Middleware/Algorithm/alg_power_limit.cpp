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


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 返回单个电机的计算功率
 * 
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @return float 
 */
float Calculate_Theoretical_Power(float omega, float torque)
{

}

/**
 * @brief 返回扭矩，单位为Nm，用返回值的时候要转化到控制量
 *
 * @param omega 转子转速，单位为rpm
 * @param power 电机功率，单位为w
 * @return float
 */
float Calculate_Toque(float omega, float power)
{
    //todo
}

    /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
