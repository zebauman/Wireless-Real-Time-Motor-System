#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float prev_error, integral;
    float out_min, out_max;
} pid_struct;

void pid_init(pid_struct *pid, float kp, float ki, float kd, float min, float max);
float pid_compute(pid_struct *pid, float target, float measured, float dt);

#endif