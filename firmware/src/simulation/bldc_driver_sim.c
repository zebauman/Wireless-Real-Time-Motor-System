#include "bldc_driver.h"
#include "motor.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <stdint.h>

LOG_MODULE_REGISTER(mock_bldc, LOG_LEVEL_INF);

/* ========================================================================= *
 * MOCK BLDC DRIVER                                                          *
 * Simulates hardware responses so the PID loop can close in software        *
 * without any physical motor or gate driver attached.                       *
 *                                                                           *
 * Physics model:                                                            *
 *   The WB55 TIM1 ARR = 3200 (64MHz / 3200 = 20kHz PWM).                  *
 *   Min meaningful pulse: ~6% of 3200 = 192 (below this = no torque)       *
 *   Max pulse: 96% of 3200 = 3072                                          *
 *   Linear RPM model: RPM = (pulse - PULSE_ZERO) * RPM_PER_TICK            *
 *   Tune PULSE_ZERO and RPM_PER_TICK to match your real motor's kV.        *
 * ========================================================================= */

/* --- Hardware model constants (calibrate to your motor) --- */
#define TIM1_ARR        3200        // Must match bldc_driver.c
#define PULSE_ZERO      192         // Pulse at which motor just starts moving
                                    // (≈ min_pwm_duty% of ARR)
#define RPM_PER_TICK    2.0f        // RPM per timer tick above PULSE_ZERO
                                    // Tune: at max pulse (3072-192=2880 ticks)
                                    // 2880 * 2.0 = 5760 RPM ≈ your MOTOR_MAX_SPEED

/* Hall sensor CW sequence for an 8-pole-pair motor (states 1-6) */
static const uint8_t HALL_SEQ_CW[6] = {1, 5, 4, 6, 2, 3};

/* ========================================================================= *
 * INTERNAL STATE                                                            *
 * ========================================================================= */
static int     sim_pulse      = 0;
static int32_t sim_rpm        = 0;
static int     hall_idx       = 0;
static uint32_t hall_last_ms  = 0;

/* ========================================================================= *
 * INIT                                                                      *
 * ========================================================================= */
int bldc_driver_init(void)
{
    LOG_INF("================================================");
    LOG_INF("  MOCK BLDC DRIVER — NO HARDWARE WILL ACTUATE  ");
    LOG_INF("  TIM1_ARR=%d  PULSE_ZERO=%d  RPM/TICK=%.1f   ",
            TIM1_ARR, PULSE_ZERO, (double)RPM_PER_TICK);
    LOG_INF("================================================");
    return 0;
}

/* ========================================================================= *
 * PWM — core mock physics                                                   *
 * ========================================================================= */
void bldc_set_pwm(int pulse)
{
    sim_pulse = pulse;

    /* Linear RPM model above the zero-crossing threshold */
    if (pulse <= PULSE_ZERO) {
        sim_rpm = 0;
    } else {
        sim_rpm = (int32_t)((pulse - PULSE_ZERO) * RPM_PER_TICK);
    }

    /* Feed simulated RPM back into the motor vault so PID loop can close */
    motor_set_speed(sim_rpm);

    /* Log only on meaningful changes to avoid flooding at 100Hz */
    static int last_logged_pulse = -999;
    if (abs(pulse - last_logged_pulse) > 50) {
        last_logged_pulse = pulse;
        LOG_INF("[MOCK PWM] pulse=%d -> sim_rpm=%d | target=%d RPM | status=0x%02X",
                pulse, sim_rpm,
                motor_get_target_speed(),
                motor_get_full_status());
    }
}

/* ========================================================================= *
 * COMMUTATION — signature matches fixed bldc_driver.h (no ccw param)       *
 * ========================================================================= */
void bldc_set_commutation(uint8_t step)
{
    /* 0 = all phases off, 7 = invalid — both are silently ignored in mock */
    if (step == 0 || step == 7) {
        return;
    }

    static uint8_t last_step = 0xFF;
    if (step != last_step) {
        last_step = step;
        LOG_DBG("[MOCK COMM] step=%d (%s)",
                step, (step > 8) ? "CCW" : "CW");
    }
}

/* ========================================================================= *
 * HALL STATE — advances index based on simulated RPM timing                *
 * ========================================================================= */
int bldc_read_hall_state(void)
{
    /* Below this threshold treat the motor as stationary */
    if (abs(sim_rpm) < 10) {
        return HALL_SEQ_CW[hall_idx];
    }

    uint32_t now = k_uptime_get_32();

    /* Time per hall step (ms):
     *   steps_per_sec = (|RPM| / 60) * POLE_PAIRS * 6
     *   ms_per_step   = 1000 / steps_per_sec
     * Add 1 to avoid division by zero at very low RPM. */
    uint32_t steps_per_sec = ((uint32_t)abs(sim_rpm) * 8U * 6U) / 60U + 1U;
    uint32_t ms_per_step   = 1000U / steps_per_sec;

    if ((now - hall_last_ms) >= ms_per_step) {
        hall_last_ms = now;
        if (sim_rpm > 0) {
            hall_idx = (hall_idx + 1) % 6;   /* CW */
        } else {
            hall_idx = (hall_idx + 5) % 6;   /* CCW (reverse walk) */
        }
    }

    return HALL_SEQ_CW[hall_idx];
}

/* ========================================================================= *
 * PERCENT → PULSE conversion (matches real driver, ARR=3200)               *
 * ========================================================================= */
int bldc_percent_to_pulse(float percent_duty_cycle)
{
    int pulse = (int)(percent_duty_cycle * 32.0f);  // 3200 / 100 = 32
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;
    return pulse;
}

/* ========================================================================= *
 * GETTERS / SETTERS                                                         *
 * ========================================================================= */

/* Returns a plausible cycle count so the PID watchdog doesn't immediately
 * time out. As long as sim_rpm != 0 the motor is "moving". */
uint32_t bldc_get_last_cycle_count(void)
{
    if (sim_rpm != 0) {
        /* Pretend a hall edge just fired to keep the watchdog happy */
        return k_cycle_get_32();
    }
    /* Return a stale value so the PID timeout fires correctly when stopped */
    return 0;
}

void bldc_set_direction(int ccw)
{
    LOG_DBG("[MOCK DIR] direction set to %s", ccw ? "CCW" : "CW");
}