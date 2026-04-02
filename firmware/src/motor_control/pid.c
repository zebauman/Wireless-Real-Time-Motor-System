#include "pid.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pid, LOG_LEVEL_INF);

/* ========================================================================= *
 * INITIALISATION                                                            *
 * ========================================================================= */
void pid_init(pid_struct *pid, float kp, float ki,
    float integral_limit, float out_min, float out_max)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->integral       = 0.0f;
    pid->integral_limit = integral_limit;
    pid->out_min        = out_min;
    pid->out_max        = out_max;

    LOG_INF("PID init: kp=%.4f  ki=%.5f  ilim=%.1f  out=[%.1f, %.1f]",
            (double)kp, (double)ki,
            (double)integral_limit,
            (double)out_min, (double)out_max);
}

/* ========================================================================= *
 * PID COMPUTE                                                               *
 * ========================================================================= */
float pid_compute(pid_struct *pid, float target, float measured, float dt)
{
    float error = target - measured;

    /* Integral accumulation with anti-windup clamp */
    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float p_term  = pid->kp * error;
    float i_term  = pid->ki * pid->integral;
    float raw_out = p_term + i_term;

    float output = raw_out;
    if      (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    LOG_DBG("tgt=%6.0f  meas=%6.0f  err=%7.1f  "
            "P=%7.2f  I=%7.2f  raw=%7.2f  out=%6.2f  integ=%7.2f",
            (double)target,   (double)measured,  (double)error,
            (double)p_term,   (double)i_term,    (double)raw_out,
            (double)output,   (double)pid->integral);

    return output;
}

/* ========================================================================= *
 * RESET                                                                     *
 * ========================================================================= */
void pid_reset(pid_struct *pid)
{
    pid->integral = 0.0f;
    LOG_INF("PID reset — integral cleared");
}