#include "bldc_driver.h"
#include "motor.h" // <-- Access to the Motor Database
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h> 
#include <stdint.h> // <-- ADDED: Ensures uint8_t and int32_t are defined

LOG_MODULE_REGISTER(mock_bldc, LOG_LEVEL_INF);

// --- INTERNAL MOCK STATE ---
static int simulated_pwm = 0;
static int simulated_rpm = 0;

int bldc_driver_init(void) {
    LOG_INF("=================================================");
    LOG_INF("   BLDC DRIVER MOCK INITIALIZED (SAFE MODE)      ");
    LOG_INF(" No actual hardware PWM or ADC will be triggered.");
    LOG_INF("=================================================");
    return 0; 
}

void bldc_set_pwm(int pulse) {
    static int last_printed_pulse = -1;
    simulated_pwm = pulse;
    
    // 1. Calculate the simulated physics
    if (pulse <= 0) {
        simulated_rpm = 0;
    } else {
        simulated_rpm = (int)((pulse - 3160) / 8.836);
    }

    // --- THE MOCK CHEAT ---
    // Feed the simulated speed back into the vault so the PID loop can close!
    motor_set_speed(simulated_rpm);

    // 2. Print the output
    if (abs(pulse - last_printed_pulse) > 50) {
        uint8_t db_status = motor_get_full_status();
        int32_t db_target = motor_get_target_speed();
        int32_t db_actual = motor_get_speed();

        LOG_INF("[MOCK PWM] Pulse: %d -> Sim Motor RPM: %d", pulse, simulated_rpm);
        LOG_INF("   ↳ [DB STATE] Status: 0x%02X | Target: %d RPM | Actual: %d RPM", 
                db_status, db_target, db_actual);
                
        last_printed_pulse = pulse;
    }
}

void bldc_set_commutation(uint8_t step, int ccw) {
    if(step == 0) return;

    static uint8_t last_step = 0xFF;
    if (step != last_step) {
        // I changed this to DEBUG level so it doesn't flood your console, 
        // but you can change it back to LOG_INF if you want to watch the steps!
        LOG_DBG("[MOCK COMM] Activating Phase Step: %d | Dir: %s", 
                 step, ccw ? "CCW" : "CW");
        last_step = step;
    }
}

int bldc_read_hall_state(void) {
    static const int hall_seq[6] = {1, 5, 4, 6, 2, 3};
    static int current_idx = 0;
    static uint32_t last_time = 0;

    uint32_t now = k_uptime_get_32();
    
    // If the simulated RPM is practically zero, don't advance the sensors
    if (abs(simulated_rpm) < 10) {
        return hall_seq[current_idx];
    }

    int abs_rpm = abs(simulated_rpm);
    if (abs_rpm == 0) abs_rpm = 1; 
    
    // Calculate ms per hall step based on fake RPM
    uint32_t ms_per_step = 1000 / ((abs_rpm * 6) / 60 + 1); 
    
    if ((now - last_time) >= ms_per_step) {
        last_time = now;
        if (simulated_rpm > 0) {
            current_idx = (current_idx + 1) % 6; // Forward
        } else {
            current_idx = (current_idx + 5) % 6; // Reverse
        }
    }

    return hall_seq[current_idx];
}

int bldc_rpm_to_pulse(int rpm) {
    return (int)((rpm * 8.836) + 3160);
}