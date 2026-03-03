#ifndef PID_H
#define PID_H

/* ========================================================================= *
 * PID Controller                                                            *
 * ========================================================================= */

typedef struct {
    float kp, ki;
    float integral;
    float integral_limit;
    float out_min, out_max;

} pid_struct;

/** @brief Initialise all PID state. Must be called before pid_compute().
 *  @param pid      Pointer to pid_struct to initialise.
 *  @param kp       Proportional gain.
 *  @param ki       Integral gain.
 */
void pid_init(pid_struct *pid, float kp, float ki, float integral_limit,
     float out_min, float out_max);

float pid_compute(pid_struct *pid, float target, float target, float measured, float dt);

float pid_reset(pid_t *pid);

#endif /* PID_H */