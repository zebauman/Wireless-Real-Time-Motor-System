#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "motor_control.h"
#include "motor.h"
#include "bldc_driver.h"
#include "pid.h"

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* ========================================================================= *
 * CONFIGURATION                                                             *
 * ========================================================================= */
#define STACK_SIZE          2048
#define PRIO_PID            5

#define PID_PERIOD_MS       10
#define DT                  0.01f

#define HALL_TIMEOUT_MS     100U

#define STALL_TIMEOUT_MS    5000U

#define RPM_FILTER_ALPHA    0.3f
#define LOG_EVERY_N_TICKS   100         // 1 second at 100Hz

/* ── PID gains ───────────────────────────────────────────────────────────── */
#define PID_KP              0.01f
#define PID_KI              0.01f
#define PID_INTEGRAL_LIMIT  500.0f
#define PID_OUT_MIN         0.0f
#define PID_OUT_MAX         96.0f

K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);
static struct k_thread pid_thread_data;

/* ========================================================================= *
 * INTERNAL STATE                                                            *
 * ========================================================================= */
static pid_struct   rpm_pid;
static float        filtered_rpm = 0.0f;
static uint32_t     stall_ms     = 0;

extern atomic_t g_motor_speed_atomic;

static void reset_control_state(void)
{
    pid_reset(&rpm_pid);
    filtered_rpm = 0.0f;
    stall_ms     = 0;
}

/* ========================================================================= *
 * PID CONTROL THREAD                                                        *
 * ========================================================================= */
static void pid_control_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("PID thread: %uHz  kp=%.3f  ki=%.4f  PP=%d  edges/rev=%d",
            1000U / PID_PERIOD_MS,
            (double)PID_KP, (double)PID_KI,
            4, 24);

    pid_init(&rpm_pid, PID_KP, PID_KI,
             PID_INTEGRAL_LIMIT, PID_OUT_MIN, PID_OUT_MAX);

    uint32_t log_tick   = 0;
    uint8_t  last_state = 0xFF;

    while (1) {

        int32_t raw_rpm = (int32_t)atomic_get(&g_motor_speed_atomic);

        uint32_t elapsed_ms = bldc_get_rpm_age_ms();
        if (elapsed_ms > HALL_TIMEOUT_MS || bldc_is_rpm_timed_out()) {
            raw_rpm = 0;
            atomic_set(&g_motor_speed_atomic, 0);
        }

        filtered_rpm = RPM_FILTER_ALPHA * (float)raw_rpm
                     + (1.0f - RPM_FILTER_ALPHA) * filtered_rpm;

        motor_set_speed(raw_rpm);
        motor_set_filtered_speed((int32_t)filtered_rpm);

        uint8_t target_state = motor_get_target_state();
        int32_t target_rpm   = motor_get_target_speed();

        if (++log_tick >= LOG_EVERY_N_TICKS) {
            log_tick = 0;
            LOG_INF("[PID] raw=%6d  filt=%6d  tgt=%6d  "
                    "age=%5ums  state=0x%02X  stall=%ums",
                    raw_rpm, (int32_t)filtered_rpm, target_rpm,
                    elapsed_ms, motor_get_full_status(), stall_ms);
        }

        if (target_rpm != 0 && raw_rpm == 0 && elapsed_ms > 500) {
            stall_ms += PID_PERIOD_MS;
            if (stall_ms >= STALL_TIMEOUT_MS) {
                LOG_ERR("STALL: tgt=%d RPM, no movement for %ums",
                        target_rpm, STALL_TIMEOUT_MS);
                motor_trigger_estop();
                motor_set_stall_warning(true);
                reset_control_state();
            }
        } else {
            stall_ms = 0;
        }

        if (target_state == MOTOR_STATE_RUNNING_SPEED) {

            if (last_state != MOTOR_STATE_RUNNING_SPEED) {
                last_state = MOTOR_STATE_RUNNING_SPEED;
                reset_control_state();  // clear integral before softstart
                bldc_set_running();
                LOG_INF("Motor START — softstart to 15%% then PID");
            }

            float duty = pid_compute(&rpm_pid,
                                     (float)target_rpm,
                                     (float) raw_rpm,
                                     DT);
            bldc_set_pwm(bldc_percent_to_pulse(duty));

        } else {

            if (last_state != target_state) {
                last_state = target_state;
                LOG_WRN("PID inactive — state=0x%02X "
                        "(0x00=stopped  0x03=estop  0x05=fault)",
                        target_state);
                bldc_set_bootstrap();
                reset_control_state();
            }
        }

        k_msleep(PID_PERIOD_MS);
    }
}

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int motor_control_init(void)
{
    LOG_INF("Initializing motor control...");

    k_thread_create(&pid_thread_data, pid_stack,
                    K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_control_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    k_thread_name_set(&pid_thread_data, "pid_ctrl");
    return 0;
}