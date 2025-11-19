
#include "motor_sim.h"
#include "bluetooth.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(motor_sim, LOG_LEVEL_INF);

/* Thread config */
#define MOTOR_SIM_STACK_SIZE 2048
#define MOTOR_SIM_PRIORITY   5

/* Safety limits for the simulation */
#define MOTOR_MAX_SPEED     6000 /* reasonable cap to avoid huge jumps */
#define MOTOR_MIN_SPEED    -6000

K_THREAD_STACK_DEFINE(motor_sim_stack, MOTOR_SIM_STACK_SIZE);
static struct k_thread motor_sim_thread;
static k_tid_t motor_sim_thread_id;

static void motor_sim_thread_fn(void *a, void *b, void *c);

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
    /* handle negatives and overflow cleanly */
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
    /* Snapshot pre-update observable state */
    int32_t prev_pos    = motor_ctx.current_position;
    int32_t prev_speed  = motor_ctx.current_speed;
    uint8_t prev_status = motor_ctx.motor_status;

    switch (motor_ctx.last_cmd) {
    case MOTOR_MODE_OFF:
        /* turn motor off, decay speed toward 0 */
        motor_ctx.motor_status = 0x00; /* OFF */
        if (motor_ctx.current_speed > 0) {
            motor_ctx.current_speed -= 25;
            if (motor_ctx.current_speed < 0)
                motor_ctx.current_speed = 0;
        } else if (motor_ctx.current_speed < 0) {
            motor_ctx.current_speed += 25;
            if (motor_ctx.current_speed > 0)
                motor_ctx.current_speed = 0;
        }
        break;

    case MOTOR_MODE_INIT:
        /* Initialization: zero the position (simulate homing) */
        motor_ctx.motor_status = 0x01;
        if (motor_ctx.current_position > 0) {
            motor_ctx.current_position -= 5;
            if (motor_ctx.current_position < 0)
                motor_ctx.current_position = 0;
        } else if (motor_ctx.current_position < 0) {
            motor_ctx.current_position += 5;
            if (motor_ctx.current_position > 0)
                motor_ctx.current_position = 0;
        }
        break;

    case MOTOR_MODE_SPEED: {
        motor_ctx.motor_status = 0x01;

        int32_t error = motor_ctx.last_target - motor_ctx.current_speed;
        int32_t dt = small_step_signed(error, 0.2f);
        if (dt != 0) {
            motor_ctx.current_speed += dt;
            motor_ctx.current_speed = clamp_speed(motor_ctx.current_speed);
        }

        if (motor_ctx.current_speed != 0) {
            motor_ctx.current_position += motor_ctx.current_speed / 12;
            normalize_angle(&motor_ctx.current_position);
        }

        break;
    }

    case MOTOR_MODE_POSITION: {
        motor_ctx.motor_status = 0x01;
        int32_t error = motor_ctx.last_target - motor_ctx.current_position;

        /* shortest rotation: map error into [-180, 180] */
        if (error > 180)  error -= 360;
        if (error < -180) error += 360;

        if (error == 0) {
            /* already at target: ensure speed=0 and status=OFF but don't force notify */
            motor_ctx.current_speed = 0;
            motor_ctx.motor_status = 0x00; /* OFF */
            /* No early notify — final comparison will decide */
            break;
        }

        /* simulated rotational speed from error (proportional) */
        motor_ctx.current_speed = clamp_speed(error * 3);

        /* step scaled from speed but preserve sign for small speeds */
        int32_t dt = small_step_signed(motor_ctx.current_speed, 0.4f);
        if (dt != 0) {
            motor_ctx.current_position += dt;
            normalize_angle(&motor_ctx.current_position);
        }

        break;
    }

    default:
        motor_ctx.motor_status = 0x00;
        break;
    }

    /* Compare snapshot -> notify only if something actually changed */
    if (motor_ctx.current_position != prev_pos ||
        motor_ctx.current_speed    != prev_speed ||
        motor_ctx.motor_status     != prev_status) {
        motor_notify_telemetry();
    }
}

/* The simulation thread function */
static void motor_sim_thread_fn(void *a, void *b, void *c)
{
    const int period_ms = 15; /* tick period in milliseconds */

    while (1) {
        motor_sim_update();
        k_msleep(period_ms);
    }
}

/* Initialize and start the sim thread */
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
