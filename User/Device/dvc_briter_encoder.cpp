#include "dvc_briter_encoder.h"

void Class_Briter_Encoder::Init()
{
    // 初始化编码器
    // 设置波特率
    // 设置分辨率
    // 设置增量方向
    // 设置CAN ID
    // 设置CAN接收回调函数
    // 设置CAN发送缓冲区
}

void Class_Briter_Encoder::Data_Process()
{
    static uint32_t Now_Time = 0;
    static uint32_t Pre_Time = 0;

    uint16_t delta_time;
    int32_t delta_encoder;
    uint32_t tmp_encoder;
    int16_t tmp_omega;

    Struct_Briter_Encoder_Can_Data *tmp_buffer = (Struct_Briter_Encoder_Can_Data *)CAN_Manage_Object->Rx_Buffer.Data;
    Now_Time = DWT_GetTimeline_us();
    delta_time = Now_Time - Pre_Time;

    memcpy(Data.Raw_Value, &tmp_buffer->Data, 4);                                                                               // 原始数据
    Data.Now_Multi_Turn_Angle = Data.Raw_Value * 360.0 / Parameter.Lsbs_Per_Encoder_Round;                                      // 求多圈角度
    Data.Now_Angle = Data.Now_Multi_Turn_Angle % 360;                                                                           // 求单圈角度
    Data.Now_Omega = (Data.Raw_Value - Data.Pre_Raw_Value) / delta_time * 1000000.0 * 360.0 / Parameter.Lsbs_Per_Encoder_Round; // 求角速度，deg/s

    // 存储预备信息
    Data.Pre_Raw_Value = Data.Raw_Value;
    Pre_Time = Now_Time;

    if (Start_Falg == 0)
        Start_Falg = 1;
}

void Class_Briter_Encoder::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断编码器是否在线
    Flag += 1;

    Data_Process();
}

void Class_Briter_Encoder::TIM_PeriodElapsedCallback()
{
}

void Class_Briter_Encoder::TIM_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过编码器数据
    if (Flag == Pre_Flag)
    {
        // 编码器断开连接
        Encoder_Status = Encoder_Status_DISABLE;
    }
    else
    {
        // 编码器保持连接
        Encoder_Status = Encoder_Status_ENABLE;
    }
    Pre_Flag = Flag;
}