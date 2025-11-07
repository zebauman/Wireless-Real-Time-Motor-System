#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>

#include "bluetooth.h"

LOG_MODULE_REGISTER(bluetooth, LOG_LEVEL_INF);

// PROJECT IS USING LITTLE-ENDIAN

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12
#define MY_COMPANY_ID 0x706D

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

struct motor_app_ctx motor_ctx;

// UUIDS FOR THE SERVICES AND CHARACTERISTICS
// Custom Service UUID: c52081ba-e90f-40e4-a99f-ccaa4fd11c15
static const struct bt_uuid_128 motor_srv_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xc52081ba, 0xe90f, 0x40e4, 0xa99f, 0xccaa4fd11c15));

// MOTOR COMMAND CHARACTERISTIC UUID: d10b46cd-412a-4d15-a7bb-092a329eed46
static struct bt_uuid_128 motor_cmd_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xd10b46cd, 0x412a, 0x4d15, 0xa7bb, 0x092a329eed46));

// MOTOR TELEMETRY CHARACTERISTIC UUID: 17da15e5-05b1-42df-8d9d-d7645d6d9293
static struct bt_uuid_128 motor_telemetry_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x17da15e5, 0x05b1, 0x42df, 0x8d9d, 0xd7645d6d9293));

static uint8_t dev_id_le[6]; // 48-bit device ID LITTLE-ENDIAN
static uint8_t msd[2 + 6]; // MANUFACTURER SPECIFIC DATA; 2 BYTES COMPANY ID + 6 BYTES DEVICE ID

// FORWARD DECLARATIONS
void motor_notify_telemetry(void);


static void build_ids(void){
	int ret = hwinfo_get_device_id(dev_id_le, sizeof(dev_id_le)); // WILL ONLY GET THE FIRST 6 BYTES
	if(ret < 0){
		LOG_ERR("Failed to get device ID from HWINFO (err %d)", ret);
		memset(dev_id_le, 0, sizeof(dev_id_le));
	}
	// BUILD MANUFACTURER SPECIFIC DATA
	sys_put_le16(MY_COMPANY_ID, &msd[0]); // COMPANY ID
	memcpy(&msd[2], dev_id_le, sizeof(dev_id_le)); // DEVICE ID
	// []

}

// WRITE CALLBACK FOR MOTOR COMMAND CHARACTERISTIC
// EXPECTS 4 BYTES: [command (1 byte)] [value (4 bytes)]
static ssize_t write_motor(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len,
			   uint16_t offset, uint8_t flags)
{
	if(offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if(len < 5) { // MINIMUM 5 BYTES REQUIRED
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	const uint8_t *data = buf;
	// PARSE COMMAND
	motor_ctx.last_cmd = data[0];
	motor_ctx.last_target = sys_get_le32(&data[1]); // 4 BYTES FOR VALUE
	LOG_INF("Received command: 0x%02x, Target value: %d", motor_ctx.last_cmd, motor_ctx.last_target);

	// TODO: IMPLEMENT MOTOR CONTROL LOGIC HERE
	switch(motor_ctx.last_cmd){
		case 0x00: // SHUTDOWN
			motor_ctx.motor_status = 0x00; // OFF
			gpio_pin_set(led_red.port, led_red.pin, 1); // RED LED ON
			gpio_pin_set(led_blue.port, led_blue.pin, 0); // BLUE LED OFF
			gpio_pin_set(led_green.port, led_green.pin, 0); // GREEN LED OFF
			break;
		case 0x01: // INIT/CALIBRATE
			motor_ctx.motor_status = 0x01; // ON
			gpio_pin_set(led_red.port, led_red.pin, 0); // RED LED OFF
			gpio_pin_set(led_blue.port, led_blue.pin, 1); // BLUE LED ON
			gpio_pin_set(led_green.port, led_green.pin, 0); // GREEN LED OFF
			break;
		case 0x02: // SPEED CONTROL
			motor_ctx.motor_status = 0x01; // ON
			gpio_pin_set(led_red.port, led_red.pin, 0); // RED LED OFF
			gpio_pin_set(led_blue.port, led_blue.pin, 0); // BLUE LED OFF
			gpio_pin_set(led_green.port, led_green.pin, 1); // GREEN LED ON
			break;
		case 0x03: // POSITION CONTROL
			motor_ctx.motor_status = 0x01; // ON
			gpio_pin_set(led_red.port, led_red.pin, 0); // RED LED OFF
			gpio_pin_set(led_blue.port, led_blue.pin, 1); // BLUE LED ON
			gpio_pin_set(led_green.port, led_green.pin, 1); // GREEN LED ON
			break;
	}

	return len;
}

// CALLBACK FOR CLIENT CHARACTERISTIC CONFIGURATION (CCC) CHANGES
static void motor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value){
	motor_ctx.notification_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notifications %s", motor_ctx.notification_enabled ? "enabled" : "disabled");
}

// DEFINE GATT CHARACTERISTICS AND SERVICES
// DEFINE THE motor_svc SERVICE
BT_GATT_SERVICE_DEFINE(motor_svc, BT_GATT_PRIMARY_SERVICE(&motor_srv_uuid),
	// MOTOR COMMAND CHARACTERISTIC - WRITE ONLY
	// FIRST is UUID, SECOND is PROPERTIES, THIRD is PERMISSIONS,
	// FOURTH is READ CALLBACK (NULL IF NOT READABLE), FIFTH is WRITE CALLBACK
	// SIXTH is USER DATA (NULL IF NONE)
	BT_GATT_CHARACTERISTIC(&motor_cmd_char_uuid.uuid,
				   BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_motor, NULL), 
	// MOTOR TELEMETRY CHARACTERISTIC - NOTIFY ONLY
	BT_GATT_CHARACTERISTIC(&motor_telemetry_char_uuid.uuid,
				   BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),   
	// CLIENT CHARACTERISTIC CONFIGURATION (CCC) - FOR ENABLING/DISABLING NOTIFICATIONS
	BT_GATT_CCC(motor_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
	);

// [STATUS (1 BYTE)] [SPEED (4 BYTES)] [POSITION (4 BYTES)] = 9 BYTES TOTAL
static inline void pack_telemetry(uint8_t out[9]){
	out[0] = motor_ctx.motor_status;
	sys_put_le32(motor_ctx.current_speed, &out[1]);
	sys_put_le32(motor_ctx.current_position, &out[5]);
}

// HELPER TO SEND TELEMETRY NOTIFICATIONS TO PHONE => BROADCAST TO ALL CONNECTED DEVICES
void motor_notify_telemetry(void)
{
	if (!motor_ctx.notification_enabled) {
		return;
	}

	uint8_t telemetry_data[9];
	pack_telemetry(telemetry_data);

	// SEND NOTIFICATION
	int err = bt_gatt_notify(NULL, &motor_svc.attrs[4],
				 telemetry_data, sizeof(telemetry_data));
	if (err) {
		LOG_ERR("Failed to send notification (err %d)", err);
	} else {
		LOG_INF("Telemetry notification sent");
	}
}



// BLUETOOTH INIT & ADVERTISING
void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	// START ADVERTISING
	struct bt_le_adv_param adv_param = {
		.options = BT_LE_ADV_OPT_CONNECTABLE,
		.interval_min = 0x20,
		.interval_max = 0x40,
		.peer = NULL,
	};
	build_ids();

	// ADVERTISING DATA - 31 BYTES MAX
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), // FLAGS 3 BYTES
		BT_DATA_BYTES(BT_DATA_UUID128_ALL,BT_UUID_128_ENCODE(0xc52081ba, 0xe90f, 0x40e4, 0xa99f, 0xccaa4fd11c15)), // MOTOR SERVICE UUID 16 BYTES + 2 BYTES LENGTH/TYPE = 18 BYTES
		BT_DATA(BT_DATA_MANUFACTURER_DATA, msd, sizeof(msd)), // MANUFACTURER SPECIFIC DATA 6 BYTES + 2 BYTES LENGTH/TYPE = 8 BYTES
	};
	// SCAN RESPONSE DATA - 31 BYTES MAX
	const struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
	} else {
		LOG_INF("Connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);
}

struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};