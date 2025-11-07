package com.remotemotorcontroller.utils

data class connectivityParams(
    // BLE SCAN SETTINGS
    val scanMode: Int = android.bluetooth.le.ScanSettings.SCAN_MODE_LOW_LATENCY, // DROP DOWN FOR OPTIONS
    val cleanupDurationMs: Long = 5_000,


    // FILTER MOTOR MCU
    val filterScanDevice: Boolean = true,

    // AUTO-RECONNECT TO DEVICE
    val autoReconnect: Boolean = true,
    val retryIntervalMs: Long = 2_000,
    val reconnectAttempts: Int = 5,
    val reconnectTimeoutMs: Long = 20_000 // TIMEOUT FOR RECONNECT ONCE DISCONNECTED FROM DEVICE
)

