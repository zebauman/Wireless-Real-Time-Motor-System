#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H
#include <stdint.h>

int bldc_driver_init(void);
void bldc_set_pwm(int pulse);
void bldc_set_commutation(uint8_t step, int ccw);
int bldc_read_hall_state(void);

/** @brief MAP THE ABSOLUTE RPM VALUE TO THE ABS. TIMER PULSE */
int bldc_rpm_to_pulse(int rpm);

/** @brief return the motor's last CPU cycle value */
uint32_t bldc_get_last_cycle_count(void);

/** @brief SET THE MOTOR CCW -> PID NEEDS TO CONTROL SEQUENCING*/
void bldc_set_direction(int ccw);

int bldc_percent_to_pulse(float percent_duty_cycle);

#endif