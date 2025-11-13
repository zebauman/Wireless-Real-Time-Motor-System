package com.remotemotorcontroller.ui

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.isVisible
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

        val navHost = supportFragmentManager.findFragmentById(R.id.navHost) as NavHostFragment
        findViewById<BottomNavigationView>(R.id.bottomNav)
            .setupWithNavController(navHost.navController)

        deviceHeader = findViewById(R.id.deviceHeader)
        liveSummary  = findViewById(R.id.liveSummary)

        deviceHeader.setOnDisconnectClick { BLEManager.disconnect() }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch {
                    (application as App).repo.settings.collect { s ->
                        BLEManager.applyConfig(
                            autoReconnectEnabled = s.ar.autoReconnect,
                            companyId            = s.ar.companyId,
                            deviceId             = s.ar.deviceId6,
                            arTimeoutMs          = s.ar.timeoutMs,
                            arRetryMs            = s.ar.retryInterval,
                            scanMode             = s.ble.scanMode,
                            cleanupDurationMs    = s.ble.cleanupDurationMs,
                            filterScanDevice     = s.ble.filterScanDevice
                        )
                    }
                }

                // Connection header
                launch {
                    BLEManager.connectedSummary.collect { cs ->
                        if (!cs.connected) {
                            deviceHeader.setConnectionTitle(getString(R.string.msg_not_connected))
                            deviceHeader.setSubtitle("")
                            liveSummary.isVisible = false
                            deviceHeader.setDisconnectVisible(false)
                        } else {
                            val name = cs.name ?: "Unknown"
                            deviceHeader.setConnectionTitle(
                                "$name • ${getString(R.string.status_connected)}"
                            )
                            deviceHeader.setSubtitle("")
                            deviceHeader.setDisconnectVisible(true)

                            liveSummary.isVisible = true
                        }
                    }
                }

                // Live summary
                launch {
                    BLEManager.telemetry.collect { t ->
                        liveSummary.setRpm(t.rpm)
                        liveSummary.setAngle(t.angle)
                    }
                }
            }
        }
    }
}
