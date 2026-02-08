package com.remotemotorcontroller.ble

import android.bluetooth.BluetoothGattCharacteristic

sealed class BleOperation : Comparable<BleOperation>{
    abstract val priority: Int  // HIGHER NUMBER = HIGHER PRIORITY
    data class Write(
        val characteristic: BluetoothGattCharacteristic,
        val payload: ByteArray,
        val writeType: Int,     // WRITE_TYPE_DEFAULT OR WRITE_TYPE_NO_RESPONSE
        override val priority: Int = 0
    ) : BleOperation() {

        override fun compareTo(other: BleOperation): Int {
            return other.priority.compareTo(this.priority)  // Descending sort where high priority is first
        }

        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (javaClass != other?.javaClass) return false

            other as Write

            if (characteristic != other.characteristic) return false
            if (!payload.contentEquals(other.payload)) return false

            return true
        }

        override fun hashCode(): Int {
            var result = characteristic.hashCode()
            result = 31 * result + payload.contentHashCode()
            return result
        }
    }
}