#include "watchdog.h"
#include <zephyr/logging/log.h>
#include "motor_sim.h"
#include "bluetooth.h"

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

// WATCHDOG TIMEOUT 2 SECONDS
#define WATCHDOG_TIMEOUT_MS 2000

static struct k_work_delayable watchdog_work;  // WORKQUEUE THREAD -> THREAD THAT FIRES AFTER DELAY

// EMERGENCY STOP FUNCTION IF WATCHDOG EXPIRES -> WILL HALT MOTOR
static void watchdog_expired(struct k_work *work){
    LOG_ERR("Watchdog Timer Expired - Connection Lost - HALTING MOTOR.");

    motor_ctx.last_target = 0;
    motor_ctx.motor_status = 0x00;

    LOG_INF("MOTOR HALTED");
}

// INIT WATCHDOG
void watchdog_init(void){
    k_work_init_delayable(&watchdog_work, K_MSEC(WATCHDOG_TIMEOUT_MS));
    LOG_INF("WATCHDOG INITIALIZED");
}

// RESET THE DELAYED THREAD
void watchdog_kick(void){
    k_work_reschedule(&watchdog_work, K_MSEC(WATCHDOG_TIMEOUT_MS));
}