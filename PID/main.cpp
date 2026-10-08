#include<iostream>
#include"PID_personal.h"
int main() {
    PID speed_pid;
    speed_pid.Init(2.5f, 0.5f, 0.01f, 100.0f);

    float target_speed = 50.0f;
    float actual_speed = 0.0f;

    for (int i = 0; i < 5; i++) {
        float output = speed_pid.compute(actual_speed,target_speed);
        std::cout << "Cycle " << i << ": output = " << output << std::endl;
        actual_speed += output * 0.1f;
        std::cout << "actual_speed : " << actual_speed << std::endl;
    }

    return 0;
}