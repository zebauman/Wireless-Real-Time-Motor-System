package com.remotemotorcontroller.ui

import android.os.Bundle
import android.view.View
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.data.LineDataSet
import com.google.android.material.button.MaterialButton
import com.remotemotorcontroller.R

class AnalyticsFragment : Fragment(R.layout.fragment_analytics) {
    private lateinit var playPause: MaterialButton

    private lateinit var dataChart: LineChart
    private lateinit var rpmData: LineDataSet
    private lateinit var angleData: LineDataSet

    private var xValue = 0f
    private val dt = 0.2f
    private val maxPoints = 600

    private var isPaused = false

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        dataChart = view.findViewById(R.id.chartTelemetry)
        playPause = view.findViewById(R.id.buttonPlayPause)

        dataChart.apply {
            description.isEnabled = false
            setTouchEnabled(true)
            setPinchZoom(true)
            axisRight.isEnabled = false
            legend.isEnabled = true
        }

        playPause.setOnClickListener {
            isPaused = !isPaused
            playPause.text = if (isPaused) getString(R.string.play) else getString(R.string.pause)
        }

        rpmData = LineDataSet(mutableListOf(), "Motor RPM").apply{
            setDrawCircles(false)
            setDrawValues(false)
            lineWidth = 2f
            mode = LineDataSet.Mode.LINEAR
            color = ContextCompat.getColor(requireContext(), R.color.black)
        }
        angleData = LineDataSet(mutableListOf(), "Motor Angle").apply{
            setDrawValues(false)
            setDrawCircles(false)
            lineWidth = 2f
            mode = LineDataSet.Mode.LINEAR
            color = ContextCompat.getColor(requireContext(), R.color.purple)
        }
    }


}
