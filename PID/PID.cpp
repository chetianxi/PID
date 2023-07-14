//
// Created by 14129 on 2023/7/14.
//

#include "PID.h"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double integral_clamp, double output_limiting, bool mode) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->last_error = 0;
    this->integral = 0;
    this->integral_clamp = integral_clamp;
    this->output_limiting = output_limiting;
    this->mode = mode;
    this->last_output = 0;
}

double PID::calculate(double setpoint, double current_point, double dt) {
    double error = setpoint - current_point;
    double derivative = (error - this->last_error) / dt;

    if (this->mode) {
        double output = this->kp*(error - this->last_error) + this->ki*error*dt + this->kd*(error - 2*this->last_error + this->last_output)/dt;
        this->last_output = output;
        output += this->last_output;
        output = std::min(output, this->output_limiting);
        output = std::max(output, -this->output_limiting);
        return output;
    }
    else {
        this->integral += error * dt;
        this->integral = std::min(this->integral, this->integral_clamp);
        this->integral = std::max(this->integral, -this->integral_clamp);

        double output = this->kp * error + this->ki * this->integral + this->kd * derivative;
        this->last_error = error;
        output = std::min(output, this->output_limiting);
        output = std::max(output, -this->output_limiting);
        return output;
    }
}