#include "motor_sim.h"
#include "bluetooth.h" // For motor_notify_telemetry
#include "motor.h"     // For the Public API
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(motor_sim, LOG_LEVEL_INF);

/* Thread config */
#define MOTOR_SIM_STACK_SIZE 2048
#define MOTOR_SIM_PRIORITY   5

/* Safety limits */
#define MOTOR_MAX_SPEED     6000 
#define MOTOR_MIN_SPEED    -6000

K_THREAD_STACK_DEFINE(motor_sim_stack, MOTOR_SIM_STACK_SIZE);
static struct k_thread motor_sim_thread;
static k_tid_t motor_sim_thread_id;

static inline int32_t small_step_signed(int32_t value, float factor)
{
    int32_t step = (int32_t)(value * factor);
    if (step == 0) {
        if (value > 0) return 1;
        if (value < 0) return -1;
        return 0;
    }
    return step;
}

/* Normalize angle to [0, 360) */
static inline void normalize_angle(int32_t *angle)
{
    int32_t a = *angle % 360;
    if (a < 0) a += 360;
    *angle = a;
}

/* clamp speed in safe range */
static inline int32_t clamp_speed(int32_t s)
{
    if (s > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (s < MOTOR_MIN_SPEED) return MOTOR_MIN_SPEED;
    return s;
}

void motor_sim_update(void)
{
    // 1. READ CURRENT STATE (Local copies for calculation)
    int32_t curr_pos    = motor_get_position();
    int32_t curr_speed  = motor_get_speed();
    uint8_t target_mode = motor_get_target_state();
    
    // Snapshots for change detection
    int32_t prev_pos    = curr_pos;
    int32_t prev_speed  = curr_speed;
    uint8_t prev_status = motor_get_full_status();

    // 2. RUN SIMULATION LOGIC
    switch (target_mode) {
    case MOTOR_STATE_STOPPED:
    case MOTOR_STATE_ESTOP: // Treat ESTOP like STOPPED for physics
        /* turn motor off, decay speed toward 0 */
        if (curr_speed > 0) {
            curr_speed -= 25;
            if (curr_speed < 0) curr_speed = 0;
        } else if (curr_speed < 0) {
            curr_speed += 25;
            if (curr_speed > 0) curr_speed = 0;
        }
        break;

    case MOTOR_STATE_RUNNING_SPEED: {
        int32_t target = motor_get_target_speed();
        int32_t error  = target - curr_speed;
        int32_t dt     = small_step_signed(error, 0.2f);

        if (dt != 0) {
            curr_speed += dt;
            curr_speed = clamp_speed(curr_speed);
        }

        // Integrate Position
        if (curr_speed != 0) {
            curr_pos += curr_speed / 12;
            normalize_angle(&curr_pos);
        }
        break;
    }

    case MOTOR_STATE_RUNNING_POS: {
        int32_t target = motor_get_target_position();
        int32_t error  = target - curr_pos;

        /* shortest rotation: map error into [-180, 180] */
        if (error > 180)  error -= 360;
        if (error < -180) error += 360;

        if (error == 0) {
            /* already at target */
            curr_speed = 0;
        } else {
            /* simulated rotational speed from error (proportional) */
            curr_speed = clamp_speed(error * 3);

            /* step scaled from speed but preserve sign for small speeds */
            int32_t dt = small_step_signed(curr_speed, 0.4f);
            if (dt != 0) {
                curr_pos += dt;
                normalize_angle(&curr_pos);
            }
        }
        break;
    }

    default:
        curr_speed = 0; // Decay to 0 if unknown state
        break;
    }

    // 3. WRITE BACK TO MOTOR API
    // Note: The Vault handles updating the internal status flags automatically!
    if (target_mode == MOTOR_STATE_RUNNING_POS) {
        motor_set_position(curr_pos); // Will set state to RUNNING_POS
        motor_set_speed(curr_speed);  // Keep speed updated for telemetry
    } else {
        motor_set_speed(curr_speed);  // Will set state to RUNNING_SPEED or STOPPED
        motor_set_position(curr_pos); // Keep position updated
    }

    // 4. NOTIFY IF CHANGED
    if (motor_get_position() != prev_pos ||
        motor_get_speed()    != prev_speed ||
        motor_get_full_status() != prev_status) {
        
        motor_notify_telemetry();
    }
}

static void motor_sim_thread_fn(void *a, void *b, void *c)
{
    const int period_ms = 15; 
    while (1) {
        motor_sim_update();
        k_msleep(period_ms);
    }
}

void motor_sim_init(void)
{
    LOG_INF("Starting motor simulation thread");
    motor_sim_thread_id = k_thread_create(
        &motor_sim_thread, motor_sim_stack, MOTOR_SIM_STACK_SIZE,
        motor_sim_thread_fn, NULL, NULL, NULL,
        MOTOR_SIM_PRIORITY, 0, K_NO_WAIT);

#if defined(CONFIG_THREAD_NAME)
    k_thread_name_set(motor_sim_thread_id, "motor_sim");
#endif
}