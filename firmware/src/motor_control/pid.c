#include "pid.h"

void pid_init(pid_struct *pid, float kp, float ki, float kd, float min, float max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->prev_error = 0; pid->integral = 0;
    pid->out_min = min; pid->out_max = max;
}

float pid_compute(pid_struct *pid, float target, float measured, float dt) {
    float error = target - measured;
    pid->integral += error * pid->ki * dt;

    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    float derivative = (error - pid->prev_error) / dt;
    float output = (pid->kp * error) + pid->integral + (pid->kd * derivative);

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;
    return output;
}