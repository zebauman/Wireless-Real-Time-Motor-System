#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "bluetooth.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED2_NODE, gpios);


static void configure_leds(void)
{
    if (!device_is_ready(led_blue.port) ||
        !device_is_ready(led_green.port) ||
        !device_is_ready(led_red.port)) {
        LOG_ERR("LED device not ready");
        return;
    }

    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
}

int main(void)
{
    configure_leds();

    LOG_INF("Starting Bluetooth Motor Control Application");

    memset(&motor_ctx, 0, sizeof(motor_ctx));

    int err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    bt_conn_cb_register(&conn_callbacks);

    while (1) {
        motor_ctx.current_speed = (motor_ctx.current_speed + 10) % 1000;
        motor_ctx.current_position = (motor_ctx.current_position + 5) % 360;
        motor_ctx.motor_status = 0x01;

        motor_notify_telemetry();
        k_sleep(K_MSEC(3000));
    }

    return 0;
}
