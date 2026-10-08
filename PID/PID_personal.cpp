
#include"PID_personal.h"

    void PID::Init(float kp,float ki,float kd,float limit){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        output_limit = limit;
        integral = 0.0f;
        last_error = 0.0f;
    }
    
    float PID::compute(float actual,float target){
        float error = target - actual;

        integral += error;
        if (integral > output_limit){
            integral  = output_limit;
        }else if (integral < -output_limit){
            integral = -output_limit;
        }

        float derivative = error - last_error;
        last_error = error;

        float output = Kp*error + Ki*integral + Kd*derivative;

        if (output > output_limit)output = output_limit;
        if (output < -output_limit)output = -output_limit;

        return output;
    }