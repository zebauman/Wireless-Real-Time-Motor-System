package com.remotemotorcontroller.ble

sealed class BleState{
    object Disconnected : BleState()
    object Scanning : BleState()

    data class Connecting (val name: String?) : BleState()

    data class Connected(
        val name: String?,
        val telemetry: Telemetry? = null  // CONTAINS the live RPM/Angle data
    ) : BleState()
}

data class Telemetry(val status: Int, val rpm: Int, val angle: Int){
    companion object{   // USING COMPANION OBJECT for INIT TO BE ABLE TO RETURN NULL IF APPLICABLE
        fun fromBytes(value: ByteArray) : Telemetry? {
            if(value.size < 9) return null
            val status = value[0].toInt() and 0xFF
            val speed = ((value[1].toInt() and 0xFF) or ((value[2].toInt() and 0xFF) shl 8) or
                    ((value[3].toInt() and 0xFF) shl 16) or ((value[4].toInt() and 0xFF) shl 24))
            val position = ((value[5].toInt() and 0xFF) or ((value[6].toInt() and 0xFF) shl 8) or
                    ((value[7]).toInt() shl 16) or ((value[8].toInt() and 0xFF) shl 24))
            return Telemetry(status, speed, position)
        }
    }
}
