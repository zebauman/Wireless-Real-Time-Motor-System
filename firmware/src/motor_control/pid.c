#include "pid.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pid, LOG_LEVEL_DBG);

/* ========================================================================= *
 * INITIALISATION                                                            *
 * ========================================================================= */
void pid_init(pid_struct *pid,
              float kp, float ki, float kd,
              float min, float max,
              float timeout) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->prev_error   = 0.0f;
    pid->integral_sum = 0.0f;

    pid->out_min = min;
    pid->out_max = max;

    pid->stall_timer   = 0.0f;
    pid->stall_timeout = timeout;
    pid->is_stalled    = 0;
}

/* ========================================================================= *
 * EMA FILTER                                                                *
 * ========================================================================= */
float filter_rpm(float new_raw_rpm, float current_filtered_rpm, float alpha) {
    return (alpha * new_raw_rpm) + ((1.0f - alpha) * current_filtered_rpm);
}

/* ========================================================================= *
 * PID COMPUTE                                                               *
 * ========================================================================= */
float pid_compute(pid_struct *pid, float target, float measured, float dt) {

    // Stall fault is latched — caller must explicitly reset is_stalled (via
    // motor_control reset_control_state) before the motor can run again.
    // No log here: the latch event already logged at detection. Logging here
    // would fire at 100Hz and flood the deferred log buffer.
    if (pid->is_stalled) {
        return 0.0f;
    }

    float error = target - measured;

    // --- Proportional term ---
    float p_term = pid->kp * error;

    // --- Integral term ---
    // Accumulate raw error*dt (units: RPM*s). Apply ki at output time only.
    // This keeps the anti-windup clamp in consistent units with the output.
    pid->integral_sum += error * dt;

    // Anti-windup: clamp the raw accumulator to the range that ki can map
    // onto [out_min, out_max], preventing unbounded wind-up while stopped.
    float max_sum = pid->out_max / pid->ki;
    float min_sum = pid->out_min / pid->ki;
    if (pid->integral_sum > max_sum) pid->integral_sum = max_sum;
    if (pid->integral_sum < min_sum) pid->integral_sum = min_sum;

    float i_term = pid->ki * pid->integral_sum;

    // --- Derivative term ---
    float derivative = (error - pid->prev_error) / dt;
    float d_term     = pid->kd * derivative;

    pid->prev_error = error;

    // --- Raw (unclamped) output ---
    float raw_output = p_term + i_term + d_term;

    // --- Stall detection (on raw output, before clamping) ---
    // Condition: controller is demanding near-max power but the motor isn't
    // moving. If this persists beyond stall_timeout, latch the fault.
    if (raw_output >= (pid->out_max * 0.9f) && measured < 10.0f) {
        pid->stall_timer += dt;
        if (pid->stall_timer >= pid->stall_timeout) {
            pid->is_stalled    = 1;
            pid->integral_sum  = 0.0f;
            LOG_ERR("Stall detected! High demand (%.1f%%) with ~0 RPM for %.1fs.",
                    raw_output, pid->stall_timeout);
            return 0.0f;
        }
    } else {
        pid->stall_timer = 0.0f;
    }

    // --- Clamp final output ---
    float output = raw_output;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    // Telemetry: Teleplot-compatible serial output for live tuning.
    // Set CONFIG_LOG_LEVEL_DBG=n in production to compile these out.
    LOG_DBG(">Target:%.0f", target);
    LOG_DBG(">Measured:%.0f", measured);
    LOG_DBG(">PWM_Output:%.2f", output * 10.0f); // Scaled x10 for graph readability

    return output;
}