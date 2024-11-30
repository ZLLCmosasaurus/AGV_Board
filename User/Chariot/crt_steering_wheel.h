#ifndef CRT_STEERING_WHEEL_H_
#define CRT_STEERING_WHEEL_H_

/* Includes ------------------------------------------------------------------*/
#include "drv_dwt.h"
#include "dvc_djimotor.h"
#include "alg_power_limit.h"
#include "dvc_briter_encoder.h"
#include "config.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef enum
{
    STEERING_WHEEL_SLEEP,
    STEERING_WHEEL_ACTIVATED,
} STEERING_WHEEL_ENABLE_T;

typedef enum
{
    ENABLE_MINOR_ARC_OPTIMIZEATION,
    DISABLE_MINOR_ARC_OPTIMIZEATION
} STEERING_WHEEL_ARC_OPTIMIZATION_T;

typedef enum
{
    ENABLE_OPTIMIZATION_STATE,
    DISABLE_OPTIMIZATION_STATE
} STEERING_WHEEL_OPTIMIZATION_STATE_T;

typedef enum
{
    ENABLE_MINOR_DEG_OPTIMIZEATION,
    DISABLE_MINOR_DEG_OPTIMIZEATION
} STEERING_WHEEL_DEG_POTIMIZATION_T;

typedef enum
{
    DIRECTION_INVERSE,
    DIRECTION_SAME
} ENCODER_DIRECTIVE_PART_DIRECTION_t;

typedef enum
{
    A_STEERING_CAN_ID_e=0x1Au,
    B_STEERING_CAN_ID_e=0x1Bu,
    C_STEERING_CAN_ID_e=0x1Cu,
    D_STEERING_CAN_ID_e=0x1Du

} Enum_Steering_Wheel_ID;

/**
 * @brief 舵轮轮组类
 * 
 */
class Class_Steering_Wheel
{
public:
    void Init();

    void CAN_RxChassisCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void CAN_RxAgvBoardCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);

    Class_DJI_Motor_C620 Motion_Motor;
    Class_DJI_Motor_C620 Directive_Motor;
    Class_Briter_Encoder Encoder;

    Class_Power_Limit Power_Limit;

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_Steering_Wheel_ID CAN_ID;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;



    float Target_Position;      //期望位置,deg,0-360
    float Target_Velocity;      //期望速度,rpm


    float Now_Position;         // 当前位置,deg,0-360
    float Now_Velocity;         // 当前速度,rpm

    int8_t invert_flag;
    STEERING_WHEEL_ENABLE_T enable;
    STEERING_WHEEL_ARC_OPTIMIZATION_T arc_optimization;
    STEERING_WHEEL_DEG_POTIMIZATION_T deg_optimization;

    const 
};



#endif // 