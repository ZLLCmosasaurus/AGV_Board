#ifndef CRT_STEERING_WHEEL_H_
#define CRT_STEERING_WHEEL_H_

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "alg_power_limit.h"

#include "config.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef enum
{
    A_STEERING_CAN_ID=0x1Au,
    B_STEERING_CAN_ID=0x1Bu,
    C_STEERING_CAN_ID=0x1Cu,
    D_STEERING_CAN_ID=0x1Du

} Enum_Steering_Wheel_ID;

/**
 * @brief 舵轮轮组类
 * 
 */
class Class_Steering_Wheel
{

    void Init();

    Class_DJI_Motor_C620 Motion_Motor;
    Class_DJI_Motor_C620 Directive_Motor;

    Class_Briter_Encoder Encoder;


    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_Steering_Wheel_ID CAN_ID;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;



    uint16_t Target_Position;   //期望位置,0-8191
    float Target_Velocity;      //期望速度,rpm

    uint16_t Now_Position;      //当前位置,0-8191
    float Now_Velocity;         //当前速度,rpm

    /*
    期望位置（0-8191）
    数值来源于控制信息

    期望速度
    数值来源于控制信息

    
     */
};



#endif // 