#include "Steering_Wheel_Task.h"
#include "FreeRTOS.h"


//应该要加互斥锁
void State_Update(Class_Steering_Wheel *steering_wheel)
{
    
}

void

    void Steering_Wheel_Task(void const *pvParameters)
{
    static float steering_wheel_dt;
    static float steering_wheel_start;
    while(1)
    {
        steering_wheel_start = DWT_GetTimeline_ms();

        State_Update(&steering_wheel);




        steering_wheel_dt = DWT_GetTimeline_ms() - steering_wheel_start;
        osDelayUntil(1);
    }
}