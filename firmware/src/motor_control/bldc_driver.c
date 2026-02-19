#include "bldc_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_bus.h>
#include "motor.h"
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(bldc_driver, LOG_LEVEL_INF);

/* --- HARDWARE CONSTANTS --- */
// Assuming an 80MHz System Clock. 80,000,000 / 4000 = 20kHz PWM frequency.
// You may need to adjust this depending on your specific STM32 chip's clock tree!
#define TIM1_ARR 4000 
#define DEADTIME_TICKS 50 // Hardware safety delay to prevent shoot-through
#define POLE_PAIRS 8


/* --- GPIO DEFINITIONS (From DeviceTree Aliases) --- */
static const struct gpio_dt_spec hall_u = GPIO_DT_SPEC_GET(DT_ALIAS(hall_u), gpios);
static const struct gpio_dt_spec hall_v = GPIO_DT_SPEC_GET(DT_ALIAS(hall_v), gpios);
static const struct gpio_dt_spec hall_w = GPIO_DT_SPEC_GET(DT_ALIAS(hall_w), gpios);

/* --- INTERRUPT STATE --- */
static struct gpio_callback hall_u_cb;
static struct gpio_callback hall_v_cb;
static struct gpio_callback hall_w_cb;
static uint32_t last_cycle_count = 0;

void hall_isr_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int bldc_driver_init(void) {
    LOG_INF("Initializing Hardware Driver (Bare-Metal Timer + GPIO)...");

    // 1. Initialize Fast Digital Hall Sensor GPIOs
    if (!gpio_is_ready_dt(&hall_u) || !gpio_is_ready_dt(&hall_v) || !gpio_is_ready_dt(&hall_w)) {
        LOG_ERR("Hall sensor GPIOs not ready!");
        return -1;
    }
    gpio_pin_configure_dt(&hall_u, GPIO_INPUT);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT);

    // 2. Enable the clock for the Advanced Motor Control Timer (TIM1)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    // 3. Configure Timer for 20kHz
    LL_TIM_SetPrescaler(TIM1, 0); 
    LL_TIM_SetAutoReload(TIM1, TIM1_ARR); 

    // 4. Configure all 3 channels for PWM Mode 1
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    // Enable preload so PWM updates don't glitch mid-cycle
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    // 5. THE MAGIC: Insert Hardware Dead-Time
    LL_TIM_OC_SetDeadTime(TIM1, DEADTIME_TICKS);

    // 6. Enable Main Output (MOE) and Start the Counter
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);

    // Configure pins to trigger an interrupt whenever they go HIGH or LOW
    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    // Initialize Zephyr callback structures
    gpio_init_callback(&hall_u_cb, hall_isr_callback, BIT(hall_u.pin));
    gpio_init_callback(&hall_v_cb, hall_isr_callback, BIT(hall_v.pin));
    gpio_init_callback(&hall_w_cb, hall_isr_callback, BIT(hall_w.pin));

    // Bind callbacks to the actual hardware ports
    gpio_add_callback_dt(&hall_u, &hall_u_cb);
    gpio_add_callback_dt(&hall_v, &hall_v_cb);
    gpio_add_callback_dt(&hall_w, &hall_w_cb);

    return 0;
}

void hall_isr_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
uint32_t now = k_cycle_get_32();
    uint32_t dt_cycles = now - last_cycle_count;
    
    // Catch the first boot cycle
    if (last_cycle_count == 0) {
        last_cycle_count = now;
        return;
    }
    last_cycle_count = now;

    uint32_t cycles_per_sec = sys_clock_hw_cycles_per_sec();
    
    // Hardware Debounce (Ignore glitches faster than ~50 microseconds)
    uint32_t dt_us = (uint32_t)(((uint64_t)dt_cycles * 1000000) / cycles_per_sec);
    if (dt_us < 50) return; 

    // Pure Integer Math! Much faster, no floating point errors.
    // 1 mech rev = 48 steps (8 pole pairs * 6 states). 60 / 48 = 5/4.
    int32_t mech_rpm = (int32_t)((cycles_per_sec * 5) / (dt_cycles * 4));
    
    // Add direction logic (1 for CW, -1 for CCW) based on hall sequence here if needed
    // mech_rpm *= direction;

    motor_set_speed(mech_rpm);
}

/* ========================================================================= *
 * SENSOR READS                                                              *
 * ========================================================================= */
int bldc_read_hall_state(void) {
    // Read the digital pins (takes mere nanoseconds instead of milliseconds!)
    int u = gpio_pin_get_dt(&hall_u);
    int v = gpio_pin_get_dt(&hall_v);
    int w = gpio_pin_get_dt(&hall_w);
    
    // Combine into a 3-bit state (1 to 6)
    return (u << 2) | (v << 1) | w;
}

/* ========================================================================= *
 * MOTOR ACTUATION                                                           *
 * ========================================================================= */
void bldc_set_pwm(int pulse) {
    // Clamp the pulse width so we don't exceed the Auto-Reload Register
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0) pulse = 0;

    // Write directly to the bare-metal Capture/Compare Registers
    TIM1->CCR1 = pulse;
    TIM1->CCR2 = pulse;
    TIM1->CCR3 = pulse;
}

void bldc_set_commutation(uint8_t step, int ccw) {
    // Clear all outputs (Disable all high and low side switches)
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    if(step == 0) return;

    // Apply the correct phases. The hardware dead-time automatically ensures
    // that the High side and Low side don't cross over and short circuit!
    switch (step) {
        // --- CLOCKWISE (States 1-6) ---
        case 1: // W+ V-
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        case 5: // U+ V-
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 4: // U+ W-
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
        case 6: // V+ W-
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        case 2: // V+ U-
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        case 3: // W+ U-
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;

        // --- COUNTER-CLOCKWISE (States 9-14) (+8 offset) ---
        case 9: // U+ V-
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 13: // W+ V-
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        case 12: // W+ U-
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;
        case 14: // V+ U-
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        case 10: // V+ W-
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        case 11: // U+ W-
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
    }
}

int bldc_rpm_to_pulse(int rpm) {
    // NOTE: You will need to recalculate your linear regression!
    // Your old math output a timer period in nanoseconds (up to 50000).
    // Now, it needs to output a value between 0 and TIM1_ARR (4000).
    // Example placeholder mapping 0-5000 RPM to 0-4000 Pulse:
    return (int)((float)rpm * (4000.0f / 5000.0f)); 
}