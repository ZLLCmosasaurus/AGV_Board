#include "dvc_briter_encoder.h"

void Class_Briter_Encoder::Init()
{
    //初始化编码器
    //设置波特率
    //设置分辨率
    //设置增量方向
    //设置CAN ID
    //设置CAN接收回调函数
    //设置CAN发送缓冲区
}

void Class_Briter_Encoder::Data_Process()
{
    
}

void Class_Briter_Encoder::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断编码器是否在线
    Flag += 1;

    Data_Process();
}

void Class_Briter_Encoder::TIM_PeriodElapsedCallback()
{
  
}

void Class_Briter_Encoder::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过编码器数据
    if (Flag == Pre_Flag)
    {
        //编码器断开连接
        Encoder_Status = Encoder_Status_DISABLE;
    }
    else
    {
        //编码器保持连接
        Encoder_Status = Encoder_Status_ENABLE;
    }
    Pre_Flag = Flag;
}