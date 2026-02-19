#include "bldc_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <soc.h>

#define PERIOD_NS (50000) // 20kHz
#define ADC_DEV_NODE DT_ALIAS(adc0)
#define SENSOR_THRESHOLD_MV 1900

static const struct device *pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const struct device *adc_dev = DEVICE_DT_GET(ADC_DEV_NODE);
static int16_t adc_buffer[1];

// Convert Raw ADC to Millivolts
static inline int32_t adc_raw_to_mv(int32_t raw) {
    // 12-bit ADC (4095 max), 3.3V Reference
    return (raw * 3300) / 4095; 
}

// Read single channel
static int read_adc_channel(uint8_t channel) {
    if (!adc_dev) return 0;
    
    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = channel,
    };
    adc_channel_setup(adc_dev, &ch_cfg);

    const struct adc_sequence seq = {
        .channels = BIT(channel),
        .buffer = adc_buffer,
        .buffer_size = sizeof(adc_buffer),
        .resolution = 12,
    };

    if (adc_read(adc_dev, &seq) != 0) return 0;
    
    return adc_raw_to_mv(adc_buffer[0]);
}

int bldc_driver_init(void) {
    if (!device_is_ready(pwm_dev)) return -1;
    if (!device_is_ready(adc_dev)) return -2;
    return 0;
}

void bldc_set_pwm(int pulse) {
    pwm_set(pwm_dev, 1, PERIOD_NS, pulse, 0);
    pwm_set(pwm_dev, 2, PERIOD_NS, pulse, 0);
    pwm_set(pwm_dev, 3, PERIOD_NS, pulse, 0);
    
    // Enable Main Output (MOE) for STM32 Timer 1
    TIM1->BDTR |= TIM_BDTR_MOE; 
}



void bldc_set_commutation(uint8_t step, int ccw) {
    // Note: The 'step' variable already contains the CW/CCW offset 
    // because motor_control.c calculates it as: (ccw << 3) | hall_state

    // Clear all outputs (Disable all high and low side switches)
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    if(step == 0) return;

    // Apply the correct phases based on teammate's mapping
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

int bldc_read_hall_state(void) {
    // Read ADC channels: A=3, B=5, C=6
    int u = (read_adc_channel(3) > SENSOR_THRESHOLD_MV) ? 1 : 0;
    int v = (read_adc_channel(5) > SENSOR_THRESHOLD_MV) ? 1 : 0;
    int w = (read_adc_channel(6) > SENSOR_THRESHOLD_MV) ? 1 : 0;
    
    // Combine into a 3-bit state (1 to 6)
    return (u << 2) | (v << 1) | w;
}

int bldc_rpm_to_pulse(int rpm) {
    // linear regression for RPM to PWM pulse width
    return (int)((rpm * 8.836) + 3160);
}