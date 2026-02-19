#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @brief Initializes PWM, ADC, PID, and starts the motor threads
 * @return 0 on success, negative error code otherwise
 */
int motor_control_init(void);

#endif