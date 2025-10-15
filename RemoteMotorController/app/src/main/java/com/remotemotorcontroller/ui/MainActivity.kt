package com.remotemotorcontroller.ui

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.recyclerview.widget.RecyclerView
import com.remotemotorcontroller.R
import com.remotemotorcontroller.adapter.DeviceAdapter
import com.remotemotorcontroller.ble.BLEManager

class MainActivity : AppCompatActivity() {
    private lateinit var bleManager: BLEManager
    private lateinit var scanButton: Button

    private lateinit var recyclerView: RecyclerView
    private lateinit var deviceAdapter: DeviceAdapter

    private var isScanning = false

    // PERMISSIONS REQUIRED
    private val requiredPermissions = arrayOf(
        Manifest.permission.BLUETOOTH_SCAN,
        Manifest.permission.ACCESS_FINE_LOCATION,
        Manifest.permission.BLUETOOTH_CONNECT,
        )

    // PERMISSION LAUNCHER
    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions -> val allGranted = permissions.all { it.value }
        if (allGranted) startScan()
        else Log.e("MainActivity", "Cannot scan: missing permissions")
    }

    @SuppressLint("MissingPermission")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        recyclerView = findViewById(R.id.deviceRecyclerView)
        deviceAdapter = DeviceAdapter(mutableListOf()){ device ->
            connectToDevice(device)
        }
        recyclerView.layoutManager = androidx.recyclerview.widget.LinearLayoutManager(this)
        recyclerView.adapter = deviceAdapter

        bleManager = BLEManager(this)
        bleManager.setDeviceFoundListener{ device ->
            runOnUiThread {
                deviceAdapter.addOrUpdateDevice(device)
            }
        }
        bleManager.setDeviceRemovedListener { device ->
            runOnUiThread {
                deviceAdapter.removeDevice(device)
            }
        }

        bleManager.setConnectionStateListener { device, connected ->
            runOnUiThread {
                if(connected){
                    Toast.makeText(this, "Connected to ${device.name ?: "Unknown"}", Toast.LENGTH_SHORT).show()
                }else{
                    Toast.makeText(this, "Disconnected from ${device.name ?: "Unknown"}", Toast.LENGTH_SHORT).show()
                }
            }
        }

        scanButton = findViewById(R.id.scanButton)
        scanButton.setOnClickListener {
            toggleScan()
        }
    }

    private fun toggleScan(){
        if(isScanning){
            stopScan()
        }else{
            checkPermissionsAndScan()
        }
    }
    private fun checkPermissionsAndScan(){
        // CHECK IF PERMISSIONS ALREADY GRANTED
        val missingPermissions = requiredPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if(missingPermissions.isNotEmpty()){
            permissionLauncher.launch(missingPermissions.toTypedArray())
        }else{
            startScan()
        }
    }

    @SuppressLint("MissingPermission")
    private fun startScan(){
        bleManager.startScan()
        scanButton.text = getString(R.string.stop_scanning)
        isScanning = true
    }

    @SuppressLint("MissingPermission")
    private fun stopScan(){
        bleManager.stopScan()
        scanButton.text = getString(R.string.start_scanning)
        isScanning = false
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice){
        bleManager.connect(device)
    }

}