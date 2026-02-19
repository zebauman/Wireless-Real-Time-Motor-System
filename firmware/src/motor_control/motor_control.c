#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "motor_control.h"
#include "motor.h"         // Your thread-safe Vault
#include "bldc_driver.h"   // The abstracted hardware layer
#include "pid.h"           // PID math

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* --- THREAD DEFINITIONS --- */
#define STACK_SIZE 1024
#define PRIO_COMMUTATION 3 // Highest priority: precise motor timing
#define PRIO_PID         5 // Medium priority: control math
#define PRIO_HALL_MON    6 // Lower priority: monitoring

K_THREAD_STACK_DEFINE(comm_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(hall_stack, STACK_SIZE);

static struct k_thread comm_thread_data;
static struct k_thread pid_thread_data;
static struct k_thread hall_thread_data;

/* --- INTERNAL CONTROL STATE --- */
static uint8_t prev_hall_state = 0;
static int counter_clockwise = 0; // 0 for CW, 1 for CCW
static pid_struct rpm_pid;
#define POLE_PAIRS 8


/* ========================================================================= *
 * 1. COMMUTATION THREAD (Replaces teammate's 'pwm_thread')                  *
 * Runs fast (5ms). Reads Hall sensors and steps the motor phases.        *
 * ========================================================================= */
void commutation_thread(void *p1, void *p2, void *p3) {
    LOG_INF("Commutation thread started.");

    while (1) {
        // Read the abstracted Hall state (returns 0-7)
        uint8_t state = bldc_read_hall_state();
        
        // Teammate's logic: bitshift CCW flag and combine with Hall state
        uint8_t comm_step = (counter_clockwise << 3) | state;

        if (state != prev_hall_state) {
            if (state == 0) {
                // Invalid state, hold previous commutation
                bldc_set_commutation(prev_hall_state, counter_clockwise);
            } else {
                // Valid step, update commutation
                prev_hall_state = state;
                bldc_set_commutation(comm_step, counter_clockwise);
            }
        }
        k_msleep(5); // Run at 200Hz
    }
}


/* ========================================================================= *
 * 2. HALL MONITOR THREAD (Calculates RPM and pushes to the Vault)           *
 * Runs at 20ms. Measures time between steps to find mechanical RPM.      *
 * ========================================================================= */
void hall_monitor_thread(void *p1, void *p2, void *p3) {
    LOG_INF("Hall monitor thread started.");

    uint8_t last_state = 0xff;
    uint32_t last_timestamp_ms = k_uptime_get_32();

    while (1) {
        uint8_t state = bldc_read_hall_state();
        uint32_t now = k_uptime_get_32();

        if (state != last_state) {
            uint32_t dt = now - last_timestamp_ms;
            last_timestamp_ms = now;

            if (dt > 0) {
                // Teammate's RPM math:
                // electrical RPM = 60000 / (dt_ms * 6 transitions)
                float erpm = 60000.0f / ((float)dt * 6.0f);
                int32_t mech_rpm = (int32_t)(erpm / (float)POLE_PAIRS + 0.5f);

                // --- INTEGRATION POINT ---
                // Push the calculated speed safely into the Vault. 
                // Bluetooth and PID will now automatically see this new value!
                motor_set_speed(mech_rpm); 
            }
            last_state = state;
        }
        k_msleep(20); // Poll at 50Hz
    }
}


/* ========================================================================= *
 * 3. PID CONTROL LOOP (Reads from the Vault and applies PWM)                *
 * Runs at 100ms. Compares Target RPM vs Actual RPM.                      *
 * ========================================================================= */
void pid_control_thread(void *p1, void *p2, void *p3) {
    LOG_INF("PID control thread started.");

    // Teammate's PID tuning parameters
    pid_init(&rpm_pid, 0.2f, 0.05f, 0.01f, 0.0f, 100.0f);

    while (1) {
        // --- INTEGRATION POINT ---
        // Ask the Vault for the latest targets and actuals
        uint8_t target_state = motor_get_target_state(); 
        int32_t target_rpm   = motor_get_target_speed();
        int32_t actual_rpm   = motor_get_speed();

        // Check if Bluetooth (or internal logic) wants the motor running
        if (target_state == MOTOR_STATE_RUNNING_SPEED) {
            
            // Compute PID
            float output = pid_compute(&rpm_pid, (float)target_rpm, (float)actual_rpm, 0.1f);
            
            // Scale and convert to pulse width (using teammate's logic)
            int rpm_pid_val = 50 * (int)output;
            int pulse = bldc_rpm_to_pulse(rpm_pid_val);
            
            // Apply to hardware
            bldc_set_pwm(pulse);

        } else {
            // State is MOTOR_STATE_STOPPED or ESTOP
            bldc_set_pwm(0); 
            
            // Optional: reset PID integrals so it doesn't wind up while stopped
            rpm_pid.integral = 0; 
        }

        k_msleep(100); // Run at 10Hz
    }
}

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int motor_control_init(void) {
    LOG_INF("Initializing Motor Control Threads...");

    k_thread_create(&comm_thread_data, comm_stack, K_THREAD_STACK_SIZEOF(comm_stack),
                    commutation_thread, NULL, NULL, NULL,
                    PRIO_COMMUTATION, 0, K_NO_WAIT);

    k_thread_create(&hall_thread_data, hall_stack, K_THREAD_STACK_SIZEOF(hall_stack),
                    hall_monitor_thread, NULL, NULL, NULL,
                    PRIO_HALL_MON, 0, K_NO_WAIT);

    k_thread_create(&pid_thread_data, pid_stack, K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_control_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    return 0;
}