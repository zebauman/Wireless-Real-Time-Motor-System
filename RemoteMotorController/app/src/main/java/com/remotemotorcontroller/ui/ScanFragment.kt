package com.remotemotorcontroller.ui

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.remotemotorcontroller.App
import com.remotemotorcontroller.R
import com.remotemotorcontroller.adapter.BleTimeDevice
import com.remotemotorcontroller.adapter.DeviceAdapter
import com.remotemotorcontroller.ble.BLEManager
import kotlinx.coroutines.launch


class ScanFragment : Fragment(R.layout.fragment_scan) {
    private lateinit var scanButton: Button
    private lateinit var recyclerView: RecyclerView
    private lateinit var deviceAdapter: DeviceAdapter

    private var isScanning = false

    private fun requiredPermissions(): Array<String> = buildList{
            add(Manifest.permission.BLUETOOTH_SCAN)
            add(Manifest.permission.BLUETOOTH_CONNECT)
            add(Manifest.permission.ACCESS_FINE_LOCATION)
    }.toTypedArray()

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        val allGranted = grants.values.all { it }
        if (!allGranted) {
            Toast.makeText(requireContext(), "Permissions not granted", Toast.LENGTH_SHORT).show()
        }else{
            startScan()
        }
    }

    @SuppressLint("MissingPermission")
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        recyclerView = view.findViewById(R.id.deviceRecyclerView)
        deviceAdapter = DeviceAdapter(mutableListOf()){ device ->
            connectToDevice(device)
        }
        recyclerView.layoutManager = LinearLayoutManager(requireContext())
        recyclerView.adapter = deviceAdapter

        BLEManager.setDeviceFoundListener{ device ->
            view.post { deviceAdapter.addOrUpdateDevice(device) }
            }

        BLEManager.setDeviceRemovedListener { device ->
            view.post { deviceAdapter.removeDevice(device) }
        }

        BLEManager.setConnectionStateListener { device, connected ->
            view.post {
                if(connected){
                    Toast.makeText(requireContext(), "Connected to ${device.name ?: "Unknown"}", Toast.LENGTH_SHORT).show()
                }else{
                    Toast.makeText(requireContext(), "Disconnected from ${device.name ?: "Unknown"}", Toast.LENGTH_SHORT).show()
                }
            }
        }

        scanButton = view.findViewById(R.id.scanButton)
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
        val missingPermissions = requiredPermissions().filter {
            ContextCompat.checkSelfPermission(requireContext(),it) != PackageManager.PERMISSION_GRANTED
        }
        if(missingPermissions.isEmpty()){
            startScan()
        }else{
            permissionLauncher.launch(missingPermissions.toTypedArray())
        }
    }

    @SuppressLint("MissingPermission")
    private fun startScan(){
        BLEManager.startScan()
        scanButton.text = "START"
        isScanning = true
    }

    @SuppressLint("MissingPermission")
    private fun stopScan(){
        BLEManager.stopScan()
        scanButton.text = "STOP"
        isScanning = false
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BleTimeDevice){
        BLEManager.connect(device)
        val repo = (requireActivity().application as App).repo
        if(device.devId != null){
            lifecycleScope.launch {
                repo.setDeviceId6(device.devId!!)
            }
        }else{
            lifecycleScope.launch {
                repo.clearDeviceId6()
            }

        }
    }
}