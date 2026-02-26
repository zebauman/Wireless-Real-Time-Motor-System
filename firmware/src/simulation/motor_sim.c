#include "motor_sim.h"
#include "bluetooth.h"
#include "motor.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(motor_sim, LOG_LEVEL_INF);

/* ========================================================================= *
 * MOTOR SIMULATOR                                                           *
 * Provides simulated motor physics on top of the real motor state machine. *
 * Runs independently of the PID thread at a slightly different rate (15ms) *
 * to avoid lock-step coupling.                                              *
 * ========================================================================= */

/* --- Thread config --- */
#define MOTOR_SIM_STACK_SIZE  2048
#define MOTOR_SIM_PRIORITY    6     // One lower than PID (5) so PID always
                                    // runs first and sim reflects its output

/* --- Physics limits --- */
#define MOTOR_MAX_SPEED       6000
#define MOTOR_MIN_SPEED      -6000
#define SPEED_DECAY_STEP      25    // RPM lost per sim tick when coasting to stop
#define SPEED_ACCEL_FACTOR    0.2f  // Fraction of error applied per tick (speed mode)
#define POS_SPEED_FACTOR      3     // RPM = error_deg * this (position mode)
#define POS_STEP_FACTOR       0.4f  // Fraction of speed applied as position step

K_THREAD_STACK_DEFINE(motor_sim_stack, MOTOR_SIM_STACK_SIZE);
static struct k_thread motor_sim_thread;
static k_tid_t         motor_sim_tid;

/* ========================================================================= *
 * HELPERS                                                                   *
 * ========================================================================= */

/** @brief Return a signed step of at least ±1, scaled by factor.
 *  Prevents the simulation from stalling at zero when error is tiny. */
static inline int32_t min_step(int32_t value, float factor)
{
    int32_t step = (int32_t)((float)value * factor);
    if (step == 0) {
        if (value > 0) return  1;
        if (value < 0) return -1;
    }
    return step;
}

/** @brief Clamp speed to safe operating range. */
static inline int32_t clamp_speed(int32_t s)
{
    if (s >  MOTOR_MAX_SPEED) return  MOTOR_MAX_SPEED;
    if (s <  MOTOR_MIN_SPEED) return  MOTOR_MIN_SPEED;
    return s;
}

/** @brief Wrap angle into [0, 360). */
static inline int32_t wrap_angle(int32_t angle)
{
    int32_t a = angle % 360;
    if (a < 0) a += 360;
    return a;
}

/* ========================================================================= *
 * SIMULATION TICK                                                           *
 * ========================================================================= */
void motor_sim_update(void)
{
    /* Snapshot current state */
    int32_t curr_speed  = motor_get_speed();
    int32_t curr_pos    = motor_get_position();
    uint8_t target_mode = motor_get_target_state();
    int32_t prev_speed  = curr_speed;
    int32_t prev_pos    = curr_pos;
    uint8_t prev_status = motor_get_full_status();

    switch (target_mode) {

    /* --------------------------------------------------------------------- *
     * STOPPED / E-STOP: decay speed toward zero, hold position              *
     * --------------------------------------------------------------------- */
    case MOTOR_STATE_STOPPED:
    case MOTOR_STATE_ESTOP:
        if (curr_speed > 0) {
            curr_speed -= SPEED_DECAY_STEP;
            if (curr_speed < 0) curr_speed = 0;
        } else if (curr_speed < 0) {
            curr_speed += SPEED_DECAY_STEP;
            if (curr_speed > 0) curr_speed = 0;
        }
        /* Still integrate position while coasting — motor doesn't teleport */
        if (curr_speed != 0) {
            curr_pos = wrap_angle(curr_pos + curr_speed / 12);
        }
        break;

    /* --------------------------------------------------------------------- *
     * SPEED MODE: converge toward target RPM, integrate position            *
     * --------------------------------------------------------------------- */
    case MOTOR_STATE_RUNNING_SPEED: {
        int32_t target = motor_get_target_speed();
        int32_t error  = target - curr_speed;
        int32_t step   = min_step(error, SPEED_ACCEL_FACTOR);

        curr_speed = clamp_speed(curr_speed + step);

        if (curr_speed != 0) {
            curr_pos = wrap_angle(curr_pos + curr_speed / 12);
        }
        break;
    }

    /* --------------------------------------------------------------------- *
     * POSITION MODE: proportional approach, slow down near target           *
     * --------------------------------------------------------------------- */
    case MOTOR_STATE_RUNNING_POS: {
        int32_t target = motor_get_target_position();
        int32_t error  = target - curr_pos;

        /* Take shortest path: wrap error into [-180, 180] */
        if (error >  180) error -= 360;
        if (error < -180) error += 360;

        if (error == 0) {
            curr_speed = 0;
        } else {
            curr_speed = clamp_speed(error * POS_SPEED_FACTOR);
            int32_t pos_step = min_step(curr_speed, POS_STEP_FACTOR);
            curr_pos = wrap_angle(curr_pos + pos_step);
        }
        break;
    }

    default:
        /* Unknown state — coast to stop */
        curr_speed = 0;
        break;
    }

    /* --------------------------------------------------------------------- *
     * Write back — always update both speed and position regardless of mode *
     * so telemetry always reflects a consistent picture.                    *
     * --------------------------------------------------------------------- */
    motor_set_speed(curr_speed);
    motor_set_position(curr_pos);

    /* --------------------------------------------------------------------- *
     * Notify only when something actually changed — avoids flooding the BLE *
     * stack with identical packets at 67Hz.                                 *
     * --------------------------------------------------------------------- */
    if (motor_get_speed()       != prev_speed  ||
        motor_get_position()    != prev_pos    ||
        motor_get_full_status() != prev_status) {
        motor_notify_telemetry();
    }
}

/* ========================================================================= *
 * THREAD                                                                    *
 * ========================================================================= */
static void motor_sim_thread_fn(void *a, void *b, void *c)
{
    /* Offset slightly from PID period (10ms) to avoid lock-step coupling.
     * 15ms gives ~67Hz sim updates with natural phase jitter vs PID. */
    const int period_ms = 15;

    LOG_INF("Motor sim thread running at ~%dHz", 1000 / period_ms);

    while (1) {
        motor_sim_update();
        k_msleep(period_ms);
    }
}

/* ========================================================================= *
 * INIT                                                                      *
 * ========================================================================= */
void motor_sim_init(void)
{
    LOG_INF("Initialising motor simulator");

    motor_sim_tid = k_thread_create(
        &motor_sim_thread, motor_sim_stack,
        K_THREAD_STACK_SIZEOF(motor_sim_stack),
        motor_sim_thread_fn, NULL, NULL, NULL,
        MOTOR_SIM_PRIORITY, 0, K_NO_WAIT);

#if defined(CONFIG_THREAD_NAME)
    k_thread_name_set(motor_sim_tid, "motor_sim");
#endif
}