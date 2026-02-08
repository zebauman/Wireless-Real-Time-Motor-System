package com.remotemotorcontroller.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothStatusCodes
import android.os.Build
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withTimeoutOrNull
import java.util.concurrent.PriorityBlockingQueue

class BleRequestQueue(
    private val scope: CoroutineScope,
    private val gattProvider: () -> BluetoothGatt?
    ) {
    private val queue = PriorityBlockingQueue<BleOperation>()

    private val callbackSignal = Mutex(locked = true)

    private var queueJob: Job? = null

    companion object {
        const val PRIORITY_CRITICAL = 100
        const val PRIORITY_HIGH = 50
        const val PRIORITY_LOW = 1
    }

    fun start() {
        if (queueJob?.isActive == true) return

        queueJob = scope.launch(Dispatchers.IO) {
            while (isActive) {
                // TAKE FROM THE QUEUE
                val op = try {
                    queue.take()
                } catch (e: InterruptedException) {
                    break;
                }

                // ATTEMPT TO EXECUTE ON THE BLE
                val gatt = gattProvider()
                if (gatt == null) {
                    Log.e("BLE", "GATT IS NULL, DROPPING EXPRESSION")
                    continue
                }

                when (op) {
                    is BleOperation.Write -> processWrite(gatt, op)
                }
            }
        }
    }

    private suspend fun processWrite(gatt: BluetoothGatt, op: BleOperation.Write){
        val isWriteWithResponse = (op.writeType == BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)

        if(!callbackSignal.isLocked) callbackSignal.tryLock()

        val success = executeWrite(gatt, op)

        if(success && isWriteWithResponse){

            withTimeoutOrNull(2000){
                callbackSignal.withLock {
                    // WAIT FOR onWriteComplete to unlock it first, only if requested a response
                }
            }
        } else if(!success){
            Log.e("BLE", "Write execution failed immediately.")
        }
        // IF WRITE WITHOUT RESPONSE -> LOOP IMMEDIATELY TO THE NEXT ITEM
    }

    fun stop(){
        queueJob?.cancel()
        queue.clear()
    }

    fun clear(){
        queue.clear()
        unlockSignal()
    }
    fun enqueueWrite(
        characteristic: BluetoothGattCharacteristic,
        data: ByteArray,
        writeType: Int = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT,
        priority: Int = PRIORITY_LOW){
        queue.add(BleOperation.Write(characteristic, data, writeType, priority))
    }

    fun onWriteComplete() {
        unlockSignal()
    }

    private fun unlockSignal() {
        if(callbackSignal.isLocked){
            try {
                callbackSignal.unlock()
            } catch (e: Exception) {

            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun executeWrite(gatt: BluetoothGatt, op: BleOperation.Write): Boolean{
        return if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU){
            val result = gatt.writeCharacteristic(op.characteristic, op.payload, op.writeType)
            result == BluetoothStatusCodes.SUCCESS
        } else {
            op.characteristic.writeType = op.writeType
            op.characteristic.value = op.payload
            gatt.writeCharacteristic(op.characteristic)
        }
    }
}