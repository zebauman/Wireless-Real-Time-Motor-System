#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H
#include <stdint.h>

int bldc_driver_init(void);
void bldc_set_pwm(int pulse);
void bldc_set_commutation(uint8_t step, int ccw);
int bldc_read_hall_state(void);
int bldc_rpm_to_pulse(int rpm);

#endif