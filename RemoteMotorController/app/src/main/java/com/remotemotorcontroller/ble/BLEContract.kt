package com.remotemotorcontroller.ble

import java.util.UUID

object BLEContract {
    val SERVICE_MOTOR: UUID = UUID.fromString("c52081ba-e90f-40e4-a99f-ccaa4fd11c15")
    val CHAR_CMD: UUID = UUID.fromString("d10b46cd-412a-4d15-a7bb-092a329eed46")

    val CHAR_HEARTBEAT: UUID = UUID.fromString("2215d558-c569-4bd1-8947-b4fd5f9432a0")
    val CHAR_TELEM: UUID = UUID.fromString("17da15e5-05b1-42df-8d9d-d7645d6d9293")

    val DESC_CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    const val CMD_SHUTDOWN:  Byte = 0x00
    const val CMD_CALIBRATE: Byte = 0x01
    const val CMD_SPEED:     Byte = 0x02
    const val CMD_POSITION:  Byte = 0x03
}