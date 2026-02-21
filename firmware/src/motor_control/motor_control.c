    #include <zephyr/kernel.h>
    #include <zephyr/logging/log.h>

    #include "motor_control.h"
    #include "motor.h"
    #include "bldc_driver.h"
    #include "pid.h"

    LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

    #define POLE_PAIRS 8    // DEFINED BY THE HARDWARE OF THE MOTOR

    /* --- THREAD DEFINITIONS --- */
    #define STACK_SIZE      1024
    #define PRIO_COMMUTATION 3 // Highest priority: precise motor timing
    #define PRIO_PID         5 // Medium priority: control math

    #define PID_PERIOD_MS   10  // PERIOD OF TIME FOR PID CTRL LOOP 1/p = hz -> 10ms period = 100Hz

    // K_THREAD_STACK_DEFINE(comm_stack, STACK_SIZE); // USING ISR THERAD NOT needed
    K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);

   // static struct k_thread comm_thread_data;
    static struct k_thread pid_thread_data;

    /* --- INTERNAL CONTROL STATE --- */
    static uint8_t prev_hall_state = 0;
    static int counter_clockwise = 0; // 0 for CW, 1 for CCW
    static pid_struct rpm_pid;


    /* ========================================================================= *
    * 2. PID CONTROL LOOP (Reads from the Vault and applies PWM)                *
    * Runs at 100ms. Compares Target RPM vs Actual RPM.                         *
    * ========================================================================= */
    void pid_control_thread(void *p1, void *p2, void *p3) {
        LOG_INF("PID control thread started.");
        uint32_t cycles_per_second = sys_clock_hw_cycles_per_sec();

        uint32_t timeout_cycles = cycles_per_second / 10;   // NUMBER OF CPU CYCLES TO TIMEOUT - 100ms timeout (1second/10)
        
        float min_pwm_duty = 6.0f;
        float max_pwm_duty = 96.0f;
        // Allow PID to output negative numbers to slow down the motor - 2 second stall timeot
        pid_init(&rpm_pid, 0.05f, 0.01f, 0.005f, min_pwm_duty, max_pwm_duty, 2.0f);

        float filtered_rpm = 0.0f;  // FROM EMA FILTER
        
        while (1) {
            // latest targets and actuals
            uint8_t target_state = motor_get_target_state(); 
            int32_t target_rpm   = motor_get_target_speed();
            int32_t actual_rpm   = motor_get_speed();

            // TIMEOUT LOGIC (WATCHDOG) TO SEE IF THE MOTOR IS ACTIVELY RUNNING/ROTATING
            uint32_t curr_cycle = k_cycle_get_32();
            uint32_t past_cycle = bldc_get_last_cycle_count();
            
            // CHECK IF TIMEOUT
            if((curr_cycle - past_cycle) > timeout_cycles){
                actual_rpm = 0;
                motor_set_speed(0);
            }
            filtered_rpm = filter_rpm((float)actual_rpm, filtered_rpm, 0.3f);

            // Check if Bluetooth (or internal logic) wants the motor running
            if (target_state == MOTOR_STATE_RUNNING_SPEED && !rpm_pid.is_stalled) {
                
                // Compute PID (dt = 10ms = 0.01s)
                float duty_cycle_percent = pid_compute(&rpm_pid, (float)target_rpm, filtered_rpm, 0.01f);
                
                int pulse = bldc_percent_to_pulse(duty_cycle_percent);
                
                // Apply to hardware
                bldc_set_pwm(pulse);

            } else {
                // State is MOTOR_STATE_STOPPED or ESTOP
                bldc_set_pwm(0); 
                
                // reset PID integrals so it doesn't wind up while stopped
                rpm_pid.integral = 0; 
                filtered_rpm = 0;
            }

            k_msleep(PID_PERIOD_MS); // Run at (1/PID_PERIOD_MS)*1000
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