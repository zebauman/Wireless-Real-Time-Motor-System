#ifndef PID_H
#define PID_H

/* ========================================================================= *
 * PID Controller                                                            *
 * ========================================================================= */

typedef struct {
    // Gains
    float kp, ki, kd;

    // State
    float prev_error;
    float integral_sum; // Raw accumulated error (units: RPM*seconds)
                        // ki is applied at compute time, NOT stored here.
                        // This keeps anti-windup clamp in the correct units.
    float out_min, out_max;

    // Stall detection
    float stall_timer;      // How long high-demand + zero-speed condition has persisted (s)
    float stall_timeout;    // Duration before latching stall fault (s)
    int   is_stalled;       // 1 = stall fault latched, 0 = normal
} pid_struct;

/** @brief Initialise all PID state. Must be called before pid_compute().
 *  @param pid      Pointer to pid_struct to initialise.
 *  @param kp       Proportional gain.
 *  @param ki       Integral gain.
 *  @param kd       Derivative gain.
 *  @param min      Minimum output clamp (e.g. 6.0 % duty cycle).
 *  @param max      Maximum output clamp (e.g. 96.0 % duty cycle).
 *  @param timeout  Seconds at max demand + ~0 RPM before stall is latched.
 */
void pid_init(pid_struct *pid,
              float kp, float ki, float kd,
              float min, float max,
              float timeout);

/** @brief Run one PID iteration.
 *  @param pid       Controller state.
 *  @param target    Desired RPM.
 *  @param measured  Filtered measured RPM.
 *  @param dt        Time since last call in seconds (e.g. 0.01 for 100 Hz).
 *  @return Duty-cycle percentage [out_min, out_max], or 0.0 if stalled.
 */
float pid_compute(pid_struct *pid, float target, float measured, float dt);

/** @brief Exponential moving average filter for RPM noise rejection.
 *  @param new_raw_rpm          Latest raw RPM sample.
 *  @param current_filtered_rpm Previous filtered value.
 *  @param alpha                Smoothing factor (0 < alpha <= 1).
 *                              Higher = more responsive, lower = smoother.
 *  @return Updated filtered RPM.
 */
float filter_rpm(float new_raw_rpm, float current_filtered_rpm, float alpha);

#endif /* PID_H */