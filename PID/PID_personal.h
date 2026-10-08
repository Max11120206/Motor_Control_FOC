#ifndef PID_personal_H     
#define PID_personal_H

struct PID
{
    /* data */
    float Kp;
    float Ki;
    float Kd;
    float last_error;
    float integral;
    float output_limit;

    // Function in .cpp file
    void Init(float kp,float ki,float kd,float limit);
    float compute(float actual,float target);
   
};

#endif