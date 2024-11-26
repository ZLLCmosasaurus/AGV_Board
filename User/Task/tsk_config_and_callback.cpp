/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**

 *
 */

/**
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "drv_dwt.h"
#include "config.h"
#include "drv_can.h"
#include "crt_steering_wheel.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_Steering_Wheel Steering_Wheel;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
#ifdef STEERING_WHEEL
/**
 * @brief
 *
 */
void Agv_Board_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        Steering_Wheel.Directive_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0x202):
    {
        Steering_Wheel.Motion_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    default:
        break;
    }
}

/**
 * @brief
 *
 */
void Agv_Board_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case Steering_Wheel.CAN_ID:
        /* code */
        break;

    default:
        break;
    }
}

#endif

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{

    DWT_Init(168);

    /********************************** 驱动层初始化 **********************************/
#ifdef STEERING_WHEEL
    /**
     * @brief
     *
     */
    CAN_Init(&hcan1, Agv_Board_CAN1_Callback);
    CAN_Init(&hcan2, Agv_Board_CAN2_Callback);

#endif // DEBUG

    /********************************* 设备层初始化 *********************************/
    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/
    Steering_Wheel.Init();
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
