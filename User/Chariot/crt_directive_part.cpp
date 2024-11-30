#include "crt_steering_wheel.h"

void Class_Steering_Wheel::CAN_RxAgvBoardCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    float motion_power, directive_power;
    // 从buffer还原两个float数据
    memcpy(&motion_power, CAN_RxMessage->Data, 4);
    memcpy(&directive_power, CAN_RxMessage->Data + 4, 4);

    switch (CAN_RxMessage->Header.StdId)
    {
    case 0x02A:
        {
            this->Power_Limit.Steering_Weel_Power[0].motion.theoretical = motion_power;
            this->Power_Limit.Steering_Weel_Power[0].directive.theoretical = directive_power;
        }
        break;
    case 0x02B:
        {
            this->Power_Limit.Steering_Weel_Power[1].motion.theoretical = motion_power;
            this->Power_Limit.Steering_Weel_Power[1].directive.theoretical = directive_power;
        }
        break;
    case 0x02C:
        {
            this->Power_Limit.Steering_Weel_Power[2].motion.theoretical = motion_power;
            this->Power_Limit.Steering_Weel_Power[2].directive.theoretical = directive_power;
        }
        break;
    case 0x02D:
        {
            this->Power_Limit.Steering_Weel_Power[3].motion.theoretical = motion_power;
            this->Power_Limit.Steering_Weel_Power[3].directive.theoretical = directive_power;
        }
        break;

    }
}

void Class_Steering_Wheel::CAN_RxChassisCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
   //todo


}


void Class_Steering_Wheel::Init()
{
    
}
