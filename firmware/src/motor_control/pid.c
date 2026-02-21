#include <stdio.h> // Include for printf
#include "pid.h"

void pid_init(pid_struct *pid, float kp, float ki, float kd, float min, float max, float timeout) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->prev_error = 0; pid->integral = 0;
    pid->out_min = min; pid->out_max = max;

    pid->stall_timer = 0.0f;
    pid->stall_timeout = timeout;
    pid->is_stalled = 0;
}

float filter_rpm(float new_raw_rpm, float current_filtered_rpm, float alpha){
    return (alpha * new_raw_rpm) + ((1.0f - alpha) * current_filtered_rpm);
}

float pid_compute(pid_struct *pid, float target, float measured, float dt) {
    if (pid->is_stalled) {
        printf("FAULT: Motor is stalled. Output disabled.\n");
        return 0.0f; 
    }

    float error = target - measured;
    
    // Calculate Integral term
    pid->integral += error * pid->ki * dt;

    // Integral Anti-windup (prevents the I-term from growing out of bounds)
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    // Calculate Derivative term
    float derivative = (error - pid->prev_error) / dt;
    
    // Calculate final terms for logging
    float p_term = pid->kp * error;
    float d_term = pid->kd * derivative;
    
    // Sum to get output
    float output = p_term + pid->integral + d_term;

    // Clamp total output
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;

    //  STALL DETECTION LOGIC -> OUTPUT IS ASKING FOR A LOT OF POWER BUT MEASURED RPM IS LIKE ZERO
    if(output >= (pid->out_max*0.9f) && measured < 10.0f){
        pid->stall_timer += dt;
        if(pid ->stall_timer >= pid->stall_timeout){
            pid->is_stalled = 1; 
            output = 0.00f;
            pid->integral = 0;
        }
    }
    else{
        pid->stall_timer = 0.0f;
    }

    printf(">Target:%.0f\n", target);
    printf(">Measured:%.0f\n", measured);
    
    // Scaling output by 10 so a 96% PWM looks like "960" on the RPM graph
    printf(">PWM_Output:%.2f\n", (output * 10.0f));
        return output;
}