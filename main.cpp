#include <iostream>
#include "../PID_test/PID/PID.h"

PID pid1(0.1,0.01,0,20,50, false);
double setpoint = 50;
double current_point = 0;
double dt = 0.1;
double output;
int main() {
    while (true) {
        output = pid1.calculate(setpoint, current_point, dt);
        current_point += output;
        if (current_point<50.55&&current_point>49.55)
        {
            break;
        }
        std::cout << "Current point: " << current_point << ", Output: " << output << std::endl;

//        if (current_point >= setpoint) {
//            // 当测量值超过设定值时，将 PID 切换至增量式 PID
//            pid1 = PID(0.1, 0.01, 0.5, 20, 50, true);
//        }
    }

    return 0;
}
