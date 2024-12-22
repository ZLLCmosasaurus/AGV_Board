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
 //#define AGV_BOARD_A //不同舵轮对应宏定义
//#define AGV_BOARD_B
//#define AGV_BOARD_C
 #define AGV_BOARD_D

#define A_ENCODER_ID 0x0AU
#define B_ENCODER_ID 0x0BU
#define C_ENCODER_ID 0x0CU
#define D_ENCODER_ID 0x0DU

#define A_STEERING_CAN_ID 0x1AU
#define B_STEERING_CAN_ID 0x1BU
#define C_STEERING_CAN_ID 0x1CU
#define D_STEERING_CAN_ID 0x1DU

#define A_BOARD_TO_OTHER_BOARDS_ID_1 0x02AU
#define B_BOARD_TO_OTHER_BOARDS_ID_1 0x02BU
#define C_BOARD_TO_OTHER_BOARDS_ID_1 0x02CU
#define D_BOARD_TO_OTHER_BOARDS_ID_1 0x02DU


#define A_BOARD_TO_OTHER_BOARDS_ID_2 0x03AU
#define B_BOARD_TO_OTHER_BOARDS_ID_2 0x03BU
#define C_BOARD_TO_OTHER_BOARDS_ID_2 0x03CU
#define D_BOARD_TO_OTHER_BOARDS_ID_2 0x03DU
// 根据板子类型定义对应的ENCODER_ID
#ifdef AGV_BOARD_A
#define ENCODER_ID A_ENCODER_ID
#define AGV_BOARD_ID A_STEERING_CAN_ID          //c板向a轮组发送的控制信息
#define BOARD_TO_BOARDS_ID_1 A_BOARD_TO_OTHER_BOARDS_ID_1  //a轮组向其他轮组发送信息的id
#define BOARD_TO_BOARDS_ID_2 A_BOARD_TO_OTHER_BOARDS_ID_2

#define AGV_BOARD_CAN_DATA_1 CAN2_0x02A_Tx_Data  
#define AGV_BOARD_CAN_DATA_2 CAN2_0x03A_Tx_Data //a轮组向其他轮组发送的信息
#define ENCODER_CAN_DATA CAN1_0x0A_Tx_Data
#elif defined(AGV_BOARD_B)
#define ENCODER_ID B_ENCODER_ID
#define AGV_BOARD_ID B_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA_1 CAN2_0x02B_Tx_Data
#define AGV_BOARD_CAN_DATA_2 CAN2_0x03B_Tx_Data
#define BOARD_TO_BOARDS_ID_1 B_BOARD_TO_OTHER_BOARDS_ID_1
#define BOARD_TO_BOARDS_ID_2 B_BOARD_TO_OTHER_BOARDS_ID_2
#define ENCODER_CAN_DATA CAN1_0x0B_Tx_Data
#elif defined(AGV_BOARD_C)
#define ENCODER_ID C_ENCODER_ID
#define AGV_BOARD_ID C_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA_1 CAN2_0x02C_Tx_Data
#define AGV_BOARD_CAN_DATA_2 CAN2_0x03C_Tx_Data
#define BOARD_TO_BOARDS_ID_1 C_BOARD_TO_OTHER_BOARDS_ID_1
#define BOARD_TO_BOARDS_ID_2 C_BOARD_TO_OTHER_BOARDS_ID_2
#define ENCODER_CAN_DATA CAN1_0x0C_Tx_Data
#elif defined(AGV_BOARD_D)
#define ENCODER_ID D_ENCODER_ID
#define AGV_BOARD_ID D_STEERING_CAN_ID
#define AGV_BOARD_CAN_DATA_1 CAN2_0x02D_Tx_Data
#define AGV_BOARD_CAN_DATA_2 CAN2_0x03D_Tx_Data

#define BOARD_TO_BOARDS_ID_1 D_BOARD_TO_OTHER_BOARDS_ID_1
#define BOARD_TO_BOARDS_ID_2 D_BOARD_TO_OTHER_BOARDS_ID_2

#define ENCODER_CAN_DATA CAN1_0x0D_Tx_Data
#else
#error "请定义舵轮板类型 (AGV_BOARD_A/B/C/D)"
#endif

#define ARM_MATH_CM3
#define STEERING_WHEEL
#define POWER_CONTROL 1

/*
    轮组数据
*/
#define ENCODER_TO_OUTPUT_RATIO 1.0f / 4.0f // 编码器转四圈，输出轴转一圈
#define OUTPUT_TO_ENCODER_RATIO 4.0f
#define ROTOR_TO_OUTPUT_RATIO 1.0f / 8.0f // 转子转八圈，输出轴转一圈
#define OUTPUT_TO_ROTOR_RATIO 8.0f

#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m

#endif

/**
 * @brief debug
 * 
 */
//#define DEBUG_DIR_SPEED //将转向轮切换为速度环PID，以便调试

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/