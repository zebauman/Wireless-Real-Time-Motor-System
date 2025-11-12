package com.remotemotorcontroller.ui

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView
import com.remotemotorcontroller.App
import com.remotemotorcontroller.R
import com.remotemotorcontroller.ble.BLEManager
import com.remotemotorcontroller.ui.widgets.DeviceHeader
import com.remotemotorcontroller.ui.widgets.LiveSummaryView
import kotlinx.coroutines.launch

class ShellActivity : AppCompatActivity() {

    private lateinit var deviceHeader: DeviceHeader
    private lateinit var liveSummary: LiveSummaryView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_shell)

        // Bottom nav <-> NavController
        val navHost = supportFragmentManager.findFragmentById(R.id.navHost) as NavHostFragment
        findViewById<BottomNavigationView>(R.id.bottomNav)
            .setupWithNavController(navHost.navController)

        deviceHeader = findViewById(R.id.deviceHeader)
        liveSummary  = findViewById(R.id.liveSummary)

        // Disconnect button behavior
        deviceHeader.setOnDisconnectClick {
            BLEManager.shutdown()
        }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED){
                (application as App).repo.settings.collect { appSettings ->
                    BLEManager.applyConfig(
                        autoReconnectEnabled = appSettings.ar.autoReconnect,
                        companyId = appSettings.ar.companyId,
                        deviceId = appSettings.ar.deviceId6,
                        arTimeoutMs = appSettings.ar.timeoutMs,
                        arRetryMs = appSettings.ar.retryInterval,
                        scanMode = appSettings.ble.scanMode,
                        cleanupDurationMs = appSettings.ble.cleanupDurationMs,
                        filterScanDevice = appSettings.ble.filterScanDevice
                    )
                }
            }
        }



    }
}
