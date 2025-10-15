package com.remotemotorcontroller.ble

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.util.Log
import androidx.annotation.RequiresPermission
import com.remotemotorcontroller.adapter.BleTimeDevice
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import java.time.Duration
import java.time.Instant

class BLEManager(private val context: Context) {
    private val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val bluetoothAdapter: BluetoothAdapter? = bluetoothManager.adapter
    private val scanner = bluetoothAdapter?.bluetoothLeScanner
    private val scannedDevices = mutableListOf<BleTimeDevice>()

    // BLE GATT CLIENT -> ALLOWS FOR CONNECTION TO BLE GATT SERVERS
    // INFORMATION REGARDING DISCOVERING SERVICES, READING, AND WRITING CHARACTERISTICS
    private var bluetoothGatt: BluetoothGatt? = null
    private var connectedDevice: BluetoothDevice? = null

    // FUNCTION TO NOTIFY THE APP WHEN CONNECTION STATE CHANGES -> LISTENER FUNCTIONS
    private var onConnectionStateChange: ((BluetoothDevice, Boolean) -> Unit)? = null

    // FUNCTION TO CALL WHEN A DEVICE IS FOUND
    private var onDeviceFound: ((BleTimeDevice) -> Unit)? = null

    // FUNCTION TO CALL WHEN A DEVICE IS TO BE REMOVED DEPRECIATED
    private var onDeviceRemoved: ((BleTimeDevice) -> Unit)? = null

    // COROUTINE SCOPE TO MANAGE BACKGROUND JOB's LIFECYCLE
    //COROUTINE is A FUNCTION THAT CAN PAUSE AND RESUME ITS EXECUTION WITHOUT BLOCKING THE THREAD
    private val coroutineScope = CoroutineScope(Dispatchers.Main + Job())
    private var cleanupJob: Job? = null

    // CALLBACK FUNCTIONS
    // CALLBACK FUNCTION FOR GATT
    private val gattCallback = object : BluetoothGattCallback() {
        // FUNCTION WHEN THE CONNECTION STATE CHANGES OF THE CONNECTED DEVICE -> MANAGED BY GATT
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if(newState == BluetoothProfile.STATE_CONNECTED){
                Log.i("BLE", "CONNECTED TO ${gatt.device?.address}")
                onConnectionStateChange?.invoke(gatt.device, true)
                gatt.discoverServices()
            }else if(newState == BluetoothProfile.STATE_DISCONNECTED){
                Log.i("BLE", "DISCONNECTED FROM ${gatt.device?.address}")
                onConnectionStateChange?.invoke(gatt.device, false)
                gatt.close()
                bluetoothGatt = null
            }
        }
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if(status == BluetoothGatt.GATT_SUCCESS){
                Log.i("BLE", "SERVICES DISCOVERED for ${gatt.device?.address}")
            }else{
                Log.e("BLE", "FAILED TO DISCOVER SERVICES for ${gatt.device?.address}")
            }
        }

    }

    // CALLBACK FUNCTION FOR BLE SCAN
    @SuppressLint("MissingPermission")
    private val leScanCallback = object : ScanCallback() {

        // Called when a device is found immediately
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device ?: return

            val existing = scannedDevices.find{ it.bDevice.address == device.address}
            val now = Instant.now()

            if(existing != null){
                existing.time = now
                existing.rssi = result.rssi
                existing.isConnectable = result.isConnectable
                // INVOKE IS UPDATING THE UI EVERY TIE THE DEVICE IS FOUND/UPDATED -> CALLBACK FUNCTION
                onDeviceFound?.invoke(existing)
            }else{
                val newDevice = BleTimeDevice(
                    device,
                    result.rssi,
                    result.isConnectable,
                    now)
                scannedDevices.add(newDevice)
                onDeviceFound?.invoke(newDevice)
            }
        }

        override fun onScanFailed(errorCode: Int){
            Log.e("BLE", "Scan failed with error code $errorCode")
        }
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun startScan() {
        if (scanner == null) {
            Log.e("BLE", "Bluetooth scanner not available.")
            return
        }
        if(!(bluetoothAdapter?.isEnabled ?: false)){
            Log.e("BLE", "Bluetooth is disabled")
            return
        }
        // DON'T START MULTIPLE JOBS -> ONLY ONE
        if(cleanupJob == null || !cleanupJob!!.isActive){
            cleanupJob = startCleanupJob()
        }

        scanner.startScan(leScanCallback)
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun stopScan() {
        scanner?.stopScan(leScanCallback)
        // CANCEL THE JOB TO STOP THE INFINITE LOOP
        cleanupJob?.cancel()
        cleanupJob = null
    }

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice){
        bluetoothGatt?.close() // CLOSE ANY PREVIOUS CONNECTIONS
        connectedDevice = device
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
    }

    // SETTERS FOR THE LISTENER FUNCTIONS
    fun setDeviceFoundListener(listener: (BleTimeDevice) -> Unit){
        onDeviceFound = listener
    }
    fun setConnectionStateListener(listener: (BluetoothDevice, Boolean) -> Unit) {
        onConnectionStateChange = listener
    }
    fun setDeviceRemovedListener(listener: (BleTimeDevice) -> Unit){
        onDeviceRemoved = listener
    }

    private fun startCleanupJob() : Job{
        return coroutineScope.launch {
            while (true) {
                delay(5000)
                val now = Instant.now()
                val timeout = Duration.ofSeconds(10).toMillis()

                val toRemove = mutableListOf<Int>()
                scannedDevices.forEachIndexed { index, device ->
                    val age = Duration.between(device.time, now).toMillis()
                    if (age > timeout) {
                        toRemove.add(index)
                    }
                }
                toRemove.reversed().forEach {
                    Log.i("BLE", "Device ${scannedDevices[it].bDevice.address} timed out")

                    onDeviceRemoved?.invoke(scannedDevices[it])
                    scannedDevices.removeAt(it)
                }
            }

        }
    }
}
