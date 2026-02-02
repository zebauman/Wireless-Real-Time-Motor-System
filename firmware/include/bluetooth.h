#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#define MY_COMPANY_ID 0x706D

// MOTOR SERVICE UUID
#define BT_UUID_MOTOR_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xc52081ba, 0xe90f, 0x40e4, 0xa99f, 0xccaa4fd11c15)

// COMMAND CHARACTERISTIC UUID
#define BT_UUID_MOTOR_CMD_VAL \
	BT_UUID_128_ENCODE(0xc52081ba, 0xe90f, 0x40e4, 0xa99f, 0xccaa4fd11c15)

// MOTOR TELEMETRY UUID
#define BT_UUID_MOTOR_TELEMETRY_VAL \ 
	BT_UUID_128_ENCODE(0x17da15e5, 0x05b1, 0x42df, 0x8d9d, 0xd7645d6d9293)

// MOTOR HEARTBEAT UUID
#define BT_UUID_MOTOR_HEARTBEAT_VAL \ 
	BT_UUID_128_ENCODE(0x2215d558, 0xc569, 0x4bd1, 0x8947, 0xb4fd5f9432a0)

// UUIDS FOR THE SERVICES AND CHARACTERISTICS
// Custom MOTOR Service
static const struct bt_uuid_128 motor_srv_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_SERVICE_VAL);

// MOTOR COMMAND CHARACTERISTIC
static struct bt_uuid_128 motor_cmd_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_CMD_VAL);

// MOTOR TELEMETRY CHARACTERISTIC
static struct bt_uuid_128 motor_telemetry_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_TELEMETRY_VAL);




enum motor_cmds{
	MOTOR_MODE_OFF = 0x00,
	MOTOR_MODE_INIT = 0x01,
	MOTOR_MODE_SPEED = 0x02,
	MOTOR_MODE_POSITION = 0x03
};

// MOTOR APPLICATION DATA STRUCTURE
struct motor_app_ctx{
	// FLAG TO INDICATE IF NOTIFICATIONS ARE ENABLED FOR USER
	uint8_t notification_enabled; 
	
	// LAST COMMAND RECEIVED FROM USER (0x00 - SHUTDOWN, 0x01 - INIT/Calibrate,
	//  0x02 - Speed Control (Set target RPM), 0x03 - POSITION CONTROL (Set target angle))
	uint8_t last_cmd;
	
	// LAST COMMAND VALUE (E.G. TARGET RPM OR TARGET ANGLE)
	int32_t last_target;

	// STATUS OF MOTOR (0x00 - OFF, 0x01 - ON, 0x02 - ERROR)
	uint8_t motor_status;

	// CURRENT SPEED OF MOTOR (RPM)
	int32_t current_speed;

	// CURRENT POSITION OF MOTOR (Degrees)
	int32_t current_position;
};

extern struct motor_app_ctx motor_ctx;

extern void watchdog_kick(void);

void bt_ready(int err);

void motor_notify_telemetry(void);
extern struct bt_conn_cb conn_callbacks;

#endif /* BLUETOOTH_H_ */
