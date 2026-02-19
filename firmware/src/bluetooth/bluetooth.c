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
#include "watchdog.h"
#include "motor.h"

LOG_MODULE_REGISTER(bluetooth, LOG_LEVEL_INF);

// PROJECT IS USING LITTLE-ENDIAN

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

static struct motor_app_ctx motor_ctx;

// UUIDS FOR THE SERVICES AND CHARACTERISTICS
// Custom MOTOR Service
static const struct bt_uuid_128 motor_srv_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_SERVICE_VAL);

// MOTOR COMMAND CHARACTERISTIC
static struct bt_uuid_128 motor_cmd_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_CMD_VAL);

// BLE WATCHDOG HEARTBEAT CHARACTERISTIC
static struct bt_uuid_128 heartbeat_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_HEARTBEAT_VAL);

// MOTOR TELEMETRY CHARACTERISTIC
static struct bt_uuid_128 motor_telemetry_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_TELEMETRY_VAL);


void watchdog_kick(void);

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
	uint8_t cmd = data[0];
	int32_t val = (int32_t) sys_get_le32(&data[1]); // 4 BYTES FOR VALUE - payload


	// DETERMINE THE NEW STATE OF THE MOTOR
	switch(cmd){
		case MOTOR_MODE_SPEED:	// SET TARGET SPEED
			motor_set_target_speed(val);
			break;

		case MOTOR_MODE_POSITION:	// SET TARGET POSITION
			motor_set_target_position(val);
			break;

		case MOTOR_MODE_INIT:
			motor_init();
			break;

		case MOTOR_MODE_OFF:
			motor_set_target_speed(0);
			break;
		default:
			LOG_WRN("UNKNOWN COMMAND: 0x%02X", cmd);
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}

// WRITE TO HEARTBEAT -> ONLY ONE BYTE 
static ssize_t write_heartbeat(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len,
			   uint16_t offset, uint8_t flags)
{
	if(offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if(len < 1) { // MINIMUM 1 BYTES REQUIRED
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *data = buf;
	uint8_t new_val = data[0];
	uint8_t old_val = motor_ctx.heartbeat_val;
	
	uint8_t diff = new_val - old_val;

	LOG_INF("Heartbeat received: %d, diff = %d", new_val, diff);


	// CASE 1: PACKET IS STALE (SAME PACKETS) => DO NOTHING, DON'T KICK THE DOG
	if(diff == 0){
		LOG_WRN("Empty heartbeat packet");
		return len;
	}
	// CASE 2: OUT OF SYNC. DIFF IS GREATER THAN 1
	else if(diff > 1){
		// RAISE WARNING FLAG -> BLE OUT OF SYNC
		motor_set_sync_warning(true);
		LOG_WRN("SYNC SLIP: %d", diff);
	}
	// CASE 3: NORMAL DIFF EQUALS 1
	else if (diff == 1){
		motor_set_sync_warning(false);
	}
	// AS LONG AS WE GET A SIGNAL FROM BLE THEN WE WILL CONTINUE THE CONNECTION EVEN WITH THE WARNINGS
	motor_ctx.heartbeat_val = new_val;
	watchdog_kick();
	
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
	// BLE->MOTOR HEARTBEAT CHARACTERISTIC
	BT_GATT_CHARACTERISTIC(&heartbeat_char_uuid.uuid,
				   BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_heartbeat, NULL), 
	// MOTOR TELEMETRY CHARACTERISTIC - NOTIFY ONLY
	BT_GATT_CHARACTERISTIC(&motor_telemetry_char_uuid.uuid,
				   BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),   
	// CLIENT CHARACTERISTIC CONFIGURATION (CCC) - FOR ENABLING/DISABLING NOTIFICATIONS
	BT_GATT_CCC(motor_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

// MERGED_BYTE (1 BYTE) = 
// [MERGED_BYTE (1 BYTE)] [SPEED (4 BYTES)] [POSITION (4 BYTES)] = 9 BYTES TOTAL
static inline void pack_telemetry(uint8_t out[9]){
	out[0] = motor_get_full_status();
	sys_put_le32(motor_get_speed(), &out[1]);
	sys_put_le32(motor_get_position(), &out[5]);
}

// HELPER TO SEND TELEMETRY NOTIFICATIONS TO PHONE => BROADCAST TO ALL CONNECTED DEVICES
void motor_notify_telemetry(void)
{
	/**
	 * ATTRIBUTES:
	 * [0] PRIMARY SERVICE
	 * [1] CHAR DECLAR. (CMD)
	 * [2] CHAR VAL. (CMD)
	 * [3] CHAR DECLAR. (HEARTBEAT)
	 * [4] CHAR VAL. (HEARTBEAT)
	 * [5] CHAR DECLAR. (TELEMETRY)
	 * [6] CHAR VAL. (TELEMETRY)
	 * [7] CCC
	 */
	if (!motor_ctx.notification_enabled) {
		return;
	}

	uint8_t telemetry_data[9];
	pack_telemetry(telemetry_data);

	// SEND NOTIFICATION
	int err = bt_gatt_notify(NULL, &motor_svc.attrs[6],
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
	motor_ctx.heartbeat_val = 0;
	motor_ctx.notification_enabled = false;

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
		BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MOTOR_SERVICE_VAL), // MOTOR SERVICE UUID 16 BYTES + 2 BYTES LENGTH/TYPE = 18 BYTES
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
		watchdog_kick();
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);
	watchdog_stop();
	motor_set_target_speed(0);
}

struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

uint8_t bt_get_heartbeat(void){
	return motor_ctx.heartbeat_val;
}

uint8_t bt_is_notify_enabled(void){
	return motor_ctx.notification_enabled;
}