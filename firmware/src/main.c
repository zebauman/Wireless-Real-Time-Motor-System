#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include "bluetooth.h"
#include "motor_sim.h"
#include <stdio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);



int main(void)
{

    LOG_INF("Starting Bluetooth Motor Control Application");    

    memset(&motor_ctx, 0, sizeof(motor_ctx));

    int err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    bt_conn_cb_register(&conn_callbacks);
    motor_sim_init();

    return 0;
}
