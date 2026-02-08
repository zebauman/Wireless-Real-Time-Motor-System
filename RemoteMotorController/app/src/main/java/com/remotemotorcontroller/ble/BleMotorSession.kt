package com.remotemotorcontroller.ble

import android.bluetooth.BluetoothGatt

class BleMotorSession(
    private val gatt: BluetoothGatt,
    private val requestQueue: BleRequestQueue
    ) {
    private val charCmd = gatt.getService(BLEContract.SERVICE_MOTOR)?.getCharacteristic(BLEContract.CHAR_CMD)
    private val charHeartbeat = gatt.getService(BLEContract.SERVICE_MOTOR)?.getCharacteristic(
        BLEContract.CHAR_HEARTBEAT)

    fun sendCommand(cmd: Byte, value: Int){
        val payload = byteArrayOf(cmd,
            (value and 0xFF).toByte(),
            ((value shr 8) and 0xFF).toByte(),
            ((value shr 16) and 0xFF).toByte(),
            ((value shr 24) and 0xFF).toByte()
        )
        charCmd?.let { requestQueue.enqueueWrite(it, payload) }
    }

    fun sendHeartBeat(count: Byte){
        charHeartbeat?.let{ requestQueue.enqueueWrite(it, byteArrayOf(count)) }
    }
}