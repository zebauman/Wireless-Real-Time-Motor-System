package com.remotemotorcontroller.ui

import android.annotation.SuppressLint
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.google.android.material.button.MaterialButton
import com.remotemotorcontroller.R
import com.remotemotorcontroller.ble.BLEManager


class ControlActivity : AppCompatActivity() {

    // ACTIVITY CONTROL UI ELEMENTS
    private lateinit var connectionText: TextView
    private lateinit var updateRpmText: TextView
    private lateinit var updateAngleText: TextView

    private lateinit var targetRpmEditText: EditText
    private lateinit var targetAngleEditText: EditText

    private lateinit var topAppBar: androidx.appcompat.widget.Toolbar

    private lateinit var startMotorButton: MaterialButton
    private lateinit var stopMotorButton: MaterialButton
    private lateinit var sendAngleButton: MaterialButton

    private lateinit var calibrateButton: Button

    // RPM CHART
    private lateinit var dataChart: LineChart
    private lateinit var rpmData: LineDataSet
    private lateinit var angleData: LineDataSet

    private var xValue = 0f
    private val dt = 0.2f
    private val maxPoints = 600

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_control)

        // INITIALIZE UI ELEMENTS
        connectionText = findViewById(R.id.textConnection)
        updateRpmText = findViewById(R.id.textRpm)
        updateAngleText = findViewById(R.id.textAngle)

        targetRpmEditText = findViewById(R.id.inputRpm)
        targetAngleEditText = findViewById(R.id.inputAngle)

        topAppBar = findViewById(R.id.topAppBar)
        topAppBar.setNavigationOnClickListener {
            finish()
        }

        startMotorButton = findViewById(R.id.buttonStartRpm)
        stopMotorButton = findViewById(R.id.buttonStopRpm)
        sendAngleButton = findViewById(R.id.buttonSendAngle)
        calibrateButton = findViewById(R.id.buttonCalibrate)

        calibrateButton.setOnClickListener{
            BLEManager.calibrate()
        }

        startMotorButton.setOnClickListener{
            val rpm = targetRpmEditText.text.toString().toIntOrNull()
            if(rpm != null){
                BLEManager.setSpeed(rpm)
                Toast.makeText(this, "Starting motor at $rpm RPM", Toast.LENGTH_SHORT).show()
            }
            else {
                Toast.makeText(this, "Invalid RPM value", Toast.LENGTH_SHORT).show()
            }
        }
        stopMotorButton.setOnClickListener{
            BLEManager.shutdown()
            Toast.makeText(this, "Stopping motor", Toast.LENGTH_SHORT).show()
        }
        sendAngleButton.setOnClickListener {
            val angle = targetAngleEditText.text.toString().toIntOrNull()
            if(angle != null){
                BLEManager.setPosition(angle)
                Toast.makeText(this, "Moving to $angle degrees", Toast.LENGTH_SHORT).show()
            }
            else {
                Toast.makeText(this, "Invalid angle value", Toast.LENGTH_SHORT).show()
            }
        }

        rpmData = LineDataSet(mutableListOf(), "Motor RPM").apply{
            setDrawCircles(false)
            setDrawValues(false)
            lineWidth = 2f
            mode = LineDataSet.Mode.LINEAR
            color = ContextCompat.getColor(this@ControlActivity, R.color.black)
        }

        angleData = LineDataSet(mutableListOf(), "Motor Angle").apply{
            setDrawValues(false)
            setDrawCircles(false)
            lineWidth = 2f
            mode = LineDataSet.Mode.LINEAR
            color = ContextCompat.getColor(this@ControlActivity, R.color.purple)
        }

        dataChart = findViewById(R.id.chartRpm)
        dataChart.data = LineData(rpmData, angleData)
        dataChart.apply{
            description.isEnabled = false
            setTouchEnabled(true)
            isDragEnabled = true
            setScaleEnabled(true)
            setPinchZoom(true)
            axisRight.isEnabled = true
            xAxis.position = XAxis.XAxisPosition.BOTTOM
            legend.isEnabled = true
        }

        BLEManager.setTelemetryListener{ status, speed: Int, position : Int ->
            runOnUiThread {
                if(BLEManager.getConectedDevice() == null){
                    connectionText.text = "Not connected"
                    return@runOnUiThread
                }
                connectionText.text = "Connected to ${BLEManager.getConnectedDeviceName()}"
                updateRpmText.text = "RPM: $speed"
                updateAngleText.text = "Angle: $position"
                appendPoint(speed, position)

            }
        }
    }

    @SuppressLint("MissingPermission")
    override fun onStart() {
        super.onStart()
        if (BLEManager.getConectedDevice() == null) {
            Log.e("BLE", "No device connected")
        }
    }

    private fun appendPoint(rpm: Int, angle: Int){
        xValue += dt
        
        rpmData.addEntry(Entry(xValue, rpm.toFloat()))
        angleData.addEntry(Entry(xValue, angle.toFloat()))
        
        if(rpmData.entryCount > maxPoints){
            rpmData.removeFirst()
        }
        if(angleData.entryCount > maxPoints){
            angleData.removeFirst()
        }

        
        dataChart.data.notifyDataChanged()
        dataChart.notifyDataSetChanged()

        dataChart.setVisibleXRangeMaximum(maxPoints * dt)
        dataChart.moveViewToX(xValue)
    }
}