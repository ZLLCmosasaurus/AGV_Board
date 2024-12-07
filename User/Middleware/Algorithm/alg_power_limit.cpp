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
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 返回单个电机的计算功率
 * 
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @return float 
 */
float Calculate_Theoretical_Power(Class_DJI_Motor_C620 (&motor)[4],Class_Power_Limit power_limit)
{
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    static float effectivePower = 0;
    // We consider motor that is just disconnected as still using power, by assuming the motor keep
    // latest output and rpm by 1 second, otherwise it is not safe if we only use energy loop
          for(int i=0;i<4;i++){
            if(measuredPower>5){
                effectivePower += motor[i].getTorqueFeedback() * rpm2av(manager.motors[i]->getRPMFeedback());
                samples[0][0] += fabsf(rpm2av(manager.motors[i]->getRPMFeedback()));
                samples[1][0] += manager.motors[i]->getTorqueFeedback() * manager.motors[i]->getTorqueFeedback();
            }
} 
        power_limit.Motor_Power.theoretical = power_limit.k1 * samples[0][0] + power_limit.k2 * samples[1][0] + effectivePower + power_limit.k3;
        params = manager.rls.update(samples, manager.measuredPower - effectivePower - manager.k3);
        manager.k1 = fmax(params[0][0], 1e-5f);  // In case the k1 diverge to negative number
        manager.k2 = fmax(params[1][0], 1e-5f);  // In case the k2 diverge to negative number
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
    const float k0 = manager.torqueConst * manager.motors[0]->getCurrentLimit() /
                     manager.motors[0]->getOutputLimit();  // torque current rate of the motor, defined as Nm/Output

    static float newTorqueCurrent[4];

    float sumCmdPower = 0.0f;
    float cmdPower[4];

    float sumError = 0.0f;
    float error[4];

    float maxPower = Utils::Math::clamp(manager.userConfiguredMaxPower, manager.fullMaxPower, manager.baseMaxPower);

    float allocatablePower = maxPower;
    float sumPowerRequired = 0.0f;
#if USE_DEBUG
    static float newCmdPower;
#endif

    for (int i = 0; i < 4; i++)
    {
        if (isMotorConnected(manager.motors[i]))
        {
            PowerObj *p = objs[i];
            cmdPower[i] = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * manager.k1 + p->pidOutput * k0 * p->pidOutput * k0 * manager.k2 +
                          manager.k3 / static_cast<float>(4);
            sumCmdPower += cmdPower[i];
            error[i] = fabs(p->setAv - p->curAv);
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
            {
                allocatablePower += -cmdPower[i];
            }
            else
            {
                sumError += error[i];
                sumPowerRequired += cmdPower[i];
            }
        }
        else if (motorDisconnectCounter[i] < 1000U)
        {
            cmdPower[i] = manager.motors[i]->getTorqueFeedback() * rpm2av(manager.motors[i]->getRPMFeedback()) +
                          fabs(rpm2av(manager.motors[i]->getRPMFeedback())) * manager.k1 +
                          manager.motors[i]->getTorqueFeedback() * manager.motors[i]->getTorqueFeedback() * manager.k2 + manager.k3 / 4.0f;
            error[i] = 0.0f;
        }
        else
        {
            cmdPower[i] = 0.0f;
            error[i]    = 0.0f;
        }
    }

    // update power status
    powerStatus.maxPowerLimited          = maxPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    if (sumCmdPower > maxPower)
    {
        float errorConfidence;
        if (sumError > error_powerDistribution_set)
        {
            errorConfidence = 1.0f;
        }
        else if (sumError > prop_powerDistribution_set)
        {
            errorConfidence =
                Utils::Math::clamp((sumError - prop_powerDistribution_set) / (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
        }
        else
        {
            errorConfidence = 0.0f;
        }
        for (int i = 0; i < 4; i++)
        {
            PowerObj *p = objs[i];//功率计算反解电流值
            if (isMotorConnected(manager.motors[i]))
            {
                if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
                {
                    newTorqueCurrent[i] = p->pidOutput;
                    continue;
                }
                float powerWeight_Error = fabs(p->setAv - p->curAv) / sumError;
                float powerWeight_Prop  = cmdPower[i] / sumPowerRequired;
                float powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
                float delta             = p->curAv * p->curAv -
                              4.0f * manager.k2 * (manager.k1 * fabs(p->curAv) + manager.k3 / static_cast<float>(4) - powerWeight * allocatablePower);
                if (floatEqual(delta, 0.0f))  // repeat roots
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
                }
                else if (delta > 0.0f)  // distinct roots
                {
                    newTorqueCurrent[i] = p->pidOutput > 0.0f ? (-p->curAv + sqrtf(delta)) / (2.0f * manager.k2) / k0
                                                              : (-p->curAv - sqrtf(delta)) / (2.0f * manager.k2) / k0;
                }
                else  // imaginary roots
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
                }
                newTorqueCurrent[i] = Utils::Math::clamp(newTorqueCurrent[i], p->pidMaxOutput);
            }
            else
            {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            if (isMotorConnected(manager.motors[i]))
            {
                newTorqueCurrent[i] = objs[i]->pidOutput;
            }
            else
            {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }

#if USE_DEBUG
    newCmdPower = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        PowerObj *p = objs[i];
        newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * manager.k1 +
                       newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * manager.k2 + manager.k3 / 4.0f;
    }
#endif

    return newTorqueCurrent;
}


    /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
