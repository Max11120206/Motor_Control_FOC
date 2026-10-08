#include<iostream>
#include<iomanip>

float clamp_current(float current,float limit){
    if (current > limit){
        return limit;
    }else if (current < -limit){
        return -limit;
    }else{
        return current;
    }
}

int main(){
    float present_current = 120.0f;
    float limit = 100.0f;
    float safe_current = clamp_current(present_current,limit);
    std::cout << "current_present:" << present_current << "A" << std::endl;
    std::cout << "current_safe:" << safe_current << "A" << std::endl;
    std::cout << std::fixed << std::setprecision(2)
              <<"Current Limitation is [-" << limit << "," << limit << "]A"
              <<std::endl;
}