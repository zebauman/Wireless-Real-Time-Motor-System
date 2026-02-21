#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float prev_error, integral;
    float out_min, out_max;

    // STALL DETECTION VARIABLES
    float stall_timer;  // HOW LONG STUCK FOR
    float stall_timeout;    // TIME BEFORE GIVING UP (TIMEOUT)
    int is_stalled;         // 1 if stalled, 0 if normal
} pid_struct;

void pid_init(pid_struct *pid, float kp, float ki, float kd, float min, float max, float timeout);
float pid_compute(pid_struct *pid, float target, float measured, float dt);
float filter_rpm(float new_raw_rpm, float current_filtered_rpm, float alpha);

#endif