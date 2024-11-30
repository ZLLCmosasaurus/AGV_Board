#ifndef BRITER
#define BRITER

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"
#include "alg_power_limit.h"
#include "dvc_dwt.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 波特率
 *
 */
typedef enum
{
    BRITER_ENCODER_SET_CAN_BAUD_RATE_500K = 0x00,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_1M,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_250K,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_125K,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_100K

} BRITER_ENCODER_CAN_BAUD_RATE_t;

/**
 * @brief 增量方向
 *
 */
typedef enum
{
    BRITER_ENCODER_INCREMENT_DIRECTION_CW = 0x00,
    BRITER_ENCODER_INCREMENT_DIRECTION_CCW

} BRITER_ENCODER_INCREMENT_DIRECTION_t;



/**
 * @brief 编码器canid
 *
 */
typedef enum
{
    A_ENCODER_ID_e = 0x0AU,
    B_ENCODER_ID_e = 0x0BU,
    C_ENCODER_ID_e = 0x0CU,
    D_ENCODER_ID_e = 0x0DU
} Enum_Encoder_ID;

/**
 * @brief 编码器状态
 *
 */
typedef enum
{
    Encoder_Status_DISABLE = 0x00,
    Encoder_Status_ENABLE
} Enum_Briter_Encoder_Status;

// 解析过的布瑞特编码器数据
typedef struct
{
    uint32_t Raw_Value;     // 最原始的反馈数据
    uint32_t Pre_Raw_Value; // 上一时刻的反馈数据

    float Now_Multi_Turn_Angle; // 编码器当前多圈角度，deg
    float Now_Angle;            // 编码器当前单圈内角度，deg
    float Now_Omega;            // 编码器当前角速度，deg/s

} Struct_Briter_Encoder_Data;

/**
 * @brief 布瑞特电机源数据
 *
 */
struct Struct_Briter_Encoder_Can_Data
{
    uint8_t Length;
    uint8_t Encoder_Address;
    uint8_t Command_Code;
    uint8_t Data[5];
} __attribute__((packed));

typedef struct
{
    uint32_t Lsbs_Per_Encoder_Round;                          // 每圈编码器分辨率
    BRITER_ENCODER_CAN_BAUD_RATE_t Baud_Rate;                 // 编码器波特率
    BRITER_ENCODER_INCREMENT_DIRECTION_t Increment_Direction; // 增量方向

} Birter_Encoder_Parameter_t;

class Class_Briter_Encoder
{
public:
    void Init();

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_PeriodElapsedCallback();
    void TIM_Alive_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_Encoder_ID CAN_ID;

    // 编码器参数
    Birter_Encoder_Parameter_t Parameter;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;

    // 当前时刻的接收flag
    uint32_t Flag = 0;
    // 前一时刻的接收flag
    uint32_t Pre_Flag = 0;
    // 编码器上电第一帧标志位
    uint8_t Start_Falg = 0;

    // 状态
    Enum_Briter_Encoder_Status Encoder_Status = Encoder_Status_DISABLE;

    // 经处理过的数据
    Struct_Briter_Encoder_Data Data;

    // 内部函数
    void Data_Process();
}

#endif // !BRITER