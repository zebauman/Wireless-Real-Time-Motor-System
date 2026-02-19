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

K_THREAD_STACK_DEFINE(comm_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);

static struct k_thread comm_thread_data;
static struct k_thread pid_thread_data;

/* --- INTERNAL CONTROL STATE --- */
static uint8_t prev_hall_state = 0;
static int counter_clockwise = 0; // 0 for CW, 1 for CCW
static pid_struct rpm_pid;
#define POLE_PAIRS 8


/* ========================================================================= *
 * 2. PID CONTROL LOOP (Reads from the Vault and applies PWM)                *
 * Runs at 100ms. Compares Target RPM vs Actual RPM.                         *
 * ========================================================================= */
void pid_control_thread(void *p1, void *p2, void *p3) {
    LOG_INF("PID control thread started.");

    // Allow PID to output negative numbers to slow down the motor
    pid_init(&rpm_pid, 0.2f, 0.05f, 0.01f, -1000.0f, 5000.0f);
    
    while (1) {
        // Ask the Vault for the latest targets and actuals
        uint8_t target_state = motor_get_target_state(); 
        int32_t target_rpm   = motor_get_target_speed();
        int32_t actual_rpm   = motor_get_speed();

        // Check if Bluetooth (or internal logic) wants the motor running
        if (target_state == MOTOR_STATE_RUNNING_SPEED) {
            
            // Compute PID
            float output = pid_compute(&rpm_pid, (float)target_rpm, (float)actual_rpm, 0.1f);
            
            // --- Feed-Forward Logic ---
            // Base speed is the target. The PID output just nudges it up or down!
            int rpm_pid_val = target_rpm + (int)output;
            
            // Safety clamps so we don't send impossible numbers to the hardware
            if (rpm_pid_val < 0) rpm_pid_val = 0;
            if (rpm_pid_val > 5000) rpm_pid_val = 5000;

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

    k_thread_create(&pid_thread_data, pid_stack, K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_control_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    return 0;
}