#ifndef MOTOR_SIM_H_
#define MOTOR_SIM_H_

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include "bluetooth.h"


void motor_sim_init(void);

// SIMULATION TICK PROCESSING
void motor_sim_update(void);

#endif /* MOTOR_SIM_H_ */