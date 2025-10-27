#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

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

void bt_ready(int err);

void motor_notify_telemetry(void);
extern struct bt_conn_cb conn_callbacks;

#endif /* BLUETOOTH_H_ */
