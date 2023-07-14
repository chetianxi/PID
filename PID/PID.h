//
// Created by 14129 on 2023/7/14.
//

#ifndef PID_TEST_PID_H
#define PID_TEST_PID_H


class PID {
public:
    PID(double kp, double ki, double kd, double integral_clamp = 0, double output_limiting = 0, bool mode = false);

    double calculate(double setpoint, double current_point, double dt);

private:
    double kp;
    double ki;
    double kd;
    double last_error;
    double integral;
    double integral_clamp;//积分限幅
    double output_limiting;//输出限幅
    bool mode;//模式 true增量式，false位置式
    double last_output;
};


#endif //PID_TEST_PID_H
