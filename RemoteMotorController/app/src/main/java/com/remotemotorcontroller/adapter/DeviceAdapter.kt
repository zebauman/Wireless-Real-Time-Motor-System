package com.remotemotorcontroller.adapter

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.remotemotorcontroller.R
import java.time.Duration
import java.time.Instant


class DeviceAdapter(
    private val devices: MutableList<BleTimeDevice>,
    private val onDeviceClick: (BluetoothDevice) -> Unit
) : RecyclerView.Adapter<DeviceAdapter.DeviceViewHolder>() {
    // ViewHolder is the object that holds the views for a single item in the list -> reuses the same views for multiple items, only showing the ones applicable
    // MANAGES THE RECYCLE VIEW

    // DEVICE VIEW HOLDER -> USE THIS CLASS FOR EACH ITEM
    inner class DeviceViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView){
        val nameView: TextView = itemView.findViewById(R.id.deviceName)
        val addressView: TextView = itemView.findViewById(R.id.deviceAddress)
        val rssiView: TextView = itemView.findViewById(R.id.deviceRsi)
        val timeView: TextView = itemView.findViewById(R.id.deviceTime)
    }

    // WHEN RECYCLERVIEW NEEDS A NEW ITEM VIEW (ROW) TO DISPLAY
    // WHEN THE LIST IS FIRST DISPLAYED (INIT ROWS) OR USER SCROLLS
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DeviceViewHolder {
        // LAYOUT INFLATER -> CREATE THE ACTUAL VIEW FROM XML TO AN ACTUAL VIEW OBJECT
        // R.layout.device
        // CONTEXT IS THE CURRENT STATE AND ENVIRONMENT OF THE APPLICATION (THEME STYLE RES ETC)
        val view = LayoutInflater.from(parent.context)
            .inflate(R.layout.device_item, parent, false)

        return DeviceViewHolder(view)
    }

    // SHOW DATA IN A SPECIFIC POSITION
    @SuppressLint("MissingPermission")
    override fun onBindViewHolder(holder: DeviceViewHolder, position: Int) {
        val device = devices[position].bDevice

        holder.nameView.text = device.name ?: "Unknown Device"
        holder.addressView.text = device.address
        holder.rssiView.text = devices[position].rssi.toString()
        holder.timeView.text = devices[position].time.toString()


        holder.itemView.setOnClickListener {
            onDeviceClick(device)
        }
    }

    override fun getItemCount(): Int {
        return devices.size
    }

    // UPDATE OR ADD (NEW OR EXISTING) DEVICES
    fun addOrUpdateDevice(newDevice: BleTimeDevice){
        val existingIndex = devices.indexOfFirst{it.bDevice.address == newDevice.bDevice.address}

        // NEW DEVICE
        if(existingIndex == -1){
            devices.add(newDevice)
            notifyItemInserted(devices.size - 1)
        }
        // EXISTING DEVICE
        else{
            devices[existingIndex] = newDevice
            notifyItemChanged(existingIndex)
        }
    }

    // REMOVE DEVICES THAT ARE NO LONGER CONNECTABLE -> TIMEOUT DEVICES
    fun removeDevice(removeDevice: BleTimeDevice){
        val existingIndex = devices.indexOfFirst{it.bDevice.address == removeDevice.bDevice.address}

        if(existingIndex != -1){
            devices.removeAt(existingIndex)
            notifyItemRemoved(existingIndex)
        }
        else{
            Log.e("BLE","Device not found in list: ${removeDevice.bDevice.address}")
        }

    }
}