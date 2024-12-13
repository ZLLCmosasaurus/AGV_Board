/**
 * @file config.h
 * @author lez
 * @brief 工程配置文件
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
// 舵小板选择
 #define AGV_BOARD_A //不同舵轮对应宏定义
// #define AGV_BOARD_B
// #define AGV_BOARD_C
// #define AGV_BOARD_D

#define A_ENCODER_ID 0x0AU
#define B_ENCODER_ID 0x0BU
#define C_ENCODER_ID 0x0CU
#define D_ENCODER_ID 0x0DU

#define A_STEERING_CAN_ID 0x1AU
#define B_STEERING_CAN_ID 0x1BU
#define C_STEERING_CAN_ID 0x1CU
#define D_STEERING_CAN_ID 0x1DU


// 根据板子类型定义对应的ENCODER_ID
#ifdef AGV_BOARD_A
#define ENCODER_ID A_ENCODER_ID
#define AGV_BOARD_ID A_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA CAN2_0x01A_Tx_Data
#elif defined(AGV_BOARD_B)
#define ENCODER_ID B_ENCODER_ID
#define AGV_BOARD_ID B_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA CAN2_0x01B_Tx_Data
#elif defined(AGV_BOARD_C)
#define ENCODER_ID C_ENCODER_ID
#define AGV_BOARD_ID C_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA CAN2_0x01C_Tx_Data
#elif defined(AGV_BOARD_D)
#define ENCODER_ID D_ENCODER_ID
#define AGV_BOARD_ID D_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA CAN2_0x01D_Tx_Data
#else
#error "请定义舵轮板类型 (AGV_BOARD_A/B/C/D)"
#endif

#define ARM_MATH_CM3
#define STEERING_WHEEL
#define POWER_CONTROL   1

/*
    轮组数据
*/
#define ENCODER_TO_OUTPUT_RATIO = 1.0f / 4.0f; // 编码器转四圈，输出轴转一圈

#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/