#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <zephyr/kernel.h>

void watchdog_init(void);

void watchdog_kick(void);

#endif