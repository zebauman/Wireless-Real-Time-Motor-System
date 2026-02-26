#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "motor_control.h"
#include "motor.h"
#include "bldc_driver.h"
#include "pid.h"

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* ========================================================================= *
 * THREAD DEFINITIONS                                                        *
 * ========================================================================= */
#define STACK_SIZE    2048
#define PRIO_PID      5       // Medium priority: control math

#define PID_PERIOD_MS 10      // 10ms period = 100Hz control loop
#define PID_PERIOD_S  0.01f   // dt passed to pid_compute()

// Timeout: if no hall edge has fired within this many ms, treat RPM as 0.
// Set slightly longer than the slowest expected hall edge interval.
#define MOTOR_STALL_TIMEOUT_MS  100U

#define RPM_FILTER_ALPHA  0.3f  // EMA coefficient: higher = more responsive,
                                // lower = smoother. Time constant ~33ms @ 100Hz.

K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);
static struct k_thread pid_thread_data;

/* ========================================================================= *
 * INTERNAL CONTROL STATE                                                    *
 * ========================================================================= */
static pid_struct rpm_pid;

/* ========================================================================= *
 * HELPERS                                                                   *
 * ========================================================================= */

/** @brief Reset all PID and filter state. Call whenever the motor stops so
 *         the next start gets a clean derivative and integral. */
static void reset_control_state(float *filtered_rpm) {
    rpm_pid.integral_sum = 0.0f;
    rpm_pid.prev_error   = 0.0f;
    rpm_pid.stall_timer  = 0.0f;
    rpm_pid.is_stalled   = 0;
    *filtered_rpm        = 0.0f;
}

/* ========================================================================= *
 * PID CONTROL THREAD                                                        *
 * Runs at 100Hz. Compares target RPM vs measured RPM and drives PWM.       *
 * ========================================================================= */
static void pid_control_thread(void *p1, void *p2, void *p3) {
    LOG_INF("PID control thread started.");

    const uint32_t cycles_per_sec   = sys_clock_hw_cycles_per_sec();
    const uint32_t timeout_cycles   = (cycles_per_sec * MOTOR_STALL_TIMEOUT_MS) / 1000U;

    const float min_pwm_duty = 6.0f;
    const float max_pwm_duty = 96.0f;

    pid_init(&rpm_pid, 0.05f, 0.01f, 0.005f, min_pwm_duty, max_pwm_duty, 2.0f);

    float filtered_rpm = 0.0f;

    while (1) {
        uint8_t target_state = motor_get_target_state();
        int32_t target_rpm   = motor_get_target_speed();
        int32_t actual_rpm   = motor_get_speed();

        // --- Hall-edge timeout: if the motor hasn't commutated recently,
        //     treat it as stopped regardless of what the speed register says.
        uint32_t curr_cycle = k_cycle_get_32();
        uint32_t past_cycle = bldc_get_last_cycle_count();

        if ((curr_cycle - past_cycle) > timeout_cycles) {
            actual_rpm = 0;
            motor_set_speed(0);
        }

        filtered_rpm = filter_rpm((float)actual_rpm, filtered_rpm, RPM_FILTER_ALPHA);

        if (target_state == MOTOR_STATE_RUNNING_SPEED && !rpm_pid.is_stalled) {
            float duty_cycle = pid_compute(&rpm_pid, (float)target_rpm,
                                           filtered_rpm, PID_PERIOD_S);
            bldc_set_pwm(bldc_percent_to_pulse(duty_cycle));

        } else {
            // Stopped, e-stop, or stall fault — cut power and reset state
            bldc_set_pwm(0);
            reset_control_state(&filtered_rpm);
        }

        k_msleep(PID_PERIOD_MS);
    }
}

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int motor_control_init(void) {
    LOG_INF("Initializing Motor Control...");

    k_thread_create(&pid_thread_data, pid_stack,
                    K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_control_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    k_thread_name_set(&pid_thread_data, "pid_ctrl");

    return 0;
}