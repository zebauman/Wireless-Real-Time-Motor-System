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
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.remotemotorcontroller.App
import com.remotemotorcontroller.R
import com.remotemotorcontroller.adapter.BleTimeDevice
import com.remotemotorcontroller.adapter.DeviceAdapter
import com.remotemotorcontroller.ble.BLEManager
import com.remotemotorcontroller.ble.BleState
import kotlinx.coroutines.launch


class ScanFragment : Fragment(R.layout.fragment_scan) {
    private lateinit var scanButton: Button
    private lateinit var recyclerView: RecyclerView
    private lateinit var deviceAdapter: DeviceAdapter

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

        scanButton = view.findViewById(R.id.scanButton)
        scanButton.setOnClickListener {
            toggleScan()
        }

        // BLE LISTENERS FOR DEVICE DISCOVERY
        BLEManager.setDeviceFoundListener{ device ->
            view.post { deviceAdapter.addOrUpdateDevice(device) }
            }

        BLEManager.setDeviceRemovedListener { device ->
            view.post { deviceAdapter.removeDevice(device) }
        }

        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED){
                BLEManager.state.collect {state ->
                    handleBleState(state)
                }
            }
        }
    }

    private fun handleBleState(state: BleState){
        when(state){
            is BleState.Connected -> {
                Toast.makeText(requireContext(),
                    "Connected to ${state.name}", Toast.LENGTH_SHORT).show()
                stopScan()
            }
            is BleState.Disconnected -> {
                // UI READY TO SCAN
            }
            is BleState.Connecting -> {
                Toast.makeText(requireContext(),
                    "Connecting...", Toast.LENGTH_SHORT).show()
            }
            else -> {}
        }
    }
    private fun toggleScan(){
        if(BLEManager.isScanning()){
            stopScan()
        }else{
            checkPermissionsAndScan()
        }
    }

    private fun checkPermissionsAndScan() {
        val permissionsNeeded = requiredPermissions()

        // Filter out permissions we already have
        val missingPermissions = permissionsNeeded.filter { permission ->
            ContextCompat.checkSelfPermission(requireContext(), permission) != PackageManager.PERMISSION_GRANTED
        }

        if (missingPermissions.isEmpty()) {
            // All permissions are already granted
            startScan()
        } else {
            // Request the missing ones
            permissionLauncher.launch(missingPermissions.toTypedArray())
        }
    }

    private fun requiredPermissions(): Array<String> {
        val permissions = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        }

        return permissions.toTypedArray()
    }

    @SuppressLint("MissingPermission")
    private fun startScan(){
        BLEManager.startScan()
        scanButton.text = "STOP SCAN"
    }

    @SuppressLint("MissingPermission")
    private fun stopScan(){
        BLEManager.stopScan()
        scanButton.text = "START SCAN"
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BleTimeDevice){
        stopScan()

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