#include "crt_steering_wheel.h"
Class_Steering_Wheel steering_wheel;
void Class_Steering_Wheel::CAN_RxAgvBoardCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{

    //
    switch (CAN_RxMessage->Header.StdId)
    {
    case 0x02A:
    {
        memcpy(&Power_Management.Motor_Data[0], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[1], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x02B:
    {
        memcpy(&Power_Management.Motor_Data[2], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[3], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x02C:
    {
        memcpy(&Power_Management.Motor_Data[4], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[5], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x02D:
    {
        memcpy(&Power_Management.Motor_Data[6], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[7], CAN_RxMessage->Data + 4, 4);
    }
    break;
    }
}

// 这里要根据帧ID判断是功率数据还是速度数据
float velocity_x, velocity_y, velocity, theta;
void Class_Steering_Wheel::CAN_RxChassisCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{


    if (CAN_RxMessage->Header.StdId == AGV_BOARD_ID)
    {
        memcpy(&velocity_x, CAN_RxMessage->Data, 4);
        memcpy(&velocity_y, CAN_RxMessage->Data + 4, 4);

        velocity = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
        theta = My_atan(velocity_y, velocity_x) + PI; // 映射到[0,2PI]

        this->Target_Velocity = velocity;
        this->Target_Angle = theta * RAD_TO_DEG;
    }

    if (CAN_RxMessage->Header.StdId == 0x01E)
    {
        memcpy(&Power_Management.Max_Power, CAN_RxMessage->Data, 2);
	  memcpy(&Power_Management.Actual_Power,CAN_RxMessage->Data+2,4);
    }
}

void Class_Steering_Wheel::Init()
{

    // todo:待调参
    Motion_Motor.PID_Omega.Init(15, 0, 0, 0, 0,16384);
    Motion_Motor.Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 14);

    Directive_Motor.PID_Angle.Init(15, 0, 0, 0, 0,16384);
    Directive_Motor.PID_Omega.Init(35, 0, 0, 0, 0,16384);

#ifdef DEBUG_DIR_SPEED
    Directive_Motor.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 8);
#else
    Directive_Motor.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_ANGLE, 8);
#endif // DEBUG

    Encoder.Init(&hcan1, static_cast<Enum_Encoder_ID>(ENCODER_ID));
}
