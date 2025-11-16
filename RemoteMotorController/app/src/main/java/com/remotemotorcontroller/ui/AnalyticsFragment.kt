package com.remotemotorcontroller.ui

import android.os.Bundle
import android.view.View
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.google.android.material.button.MaterialButton
import com.remotemotorcontroller.R
import com.remotemotorcontroller.adapter.AnalyticsViewModel
import com.remotemotorcontroller.ble.BLEManager
import kotlinx.coroutines.launch

class AnalyticsFragment : Fragment(R.layout.fragment_analytics) {

    private lateinit var chart: LineChart
    private lateinit var startStopBtn: MaterialButton

    private lateinit var rpmData: LineDataSet
    private lateinit var angleData: LineDataSet

    private var paused = false
    private var xValue = 0f

    private val viewModel: AnalyticsViewModel by activityViewModels()

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        chart = view.findViewById(R.id.chartTelemetry)
        startStopBtn = view.findViewById(R.id.buttonPlayPause)

        // Chart setup
        chart.apply {
            description.isEnabled = false
            setTouchEnabled(true)
            isDragEnabled = true
            setScaleEnabled(true)
            setPinchZoom(true)
            axisRight.isEnabled = false
            xAxis.position = XAxis.XAxisPosition.BOTTOM
            legend.isEnabled = true
        }

        rpmData = LineDataSet(viewModel.rpmEntries, "RPM").apply {
            setDrawCircles(false)
            setDrawValues(false)
            lineWidth = 2f
            mode = LineDataSet.Mode.LINEAR
            color = ContextCompat.getColor(requireContext(), R.color.black)
        }
        angleData = LineDataSet(viewModel.angleEntries, "Angle").apply {
            setDrawValues(false)
            setDrawCircles(false)
            lineWidth = 2f
            mode = LineDataSet.Mode.LINEAR
            color = ContextCompat.getColor(requireContext(), R.color.purple)
        }
        chart.data = LineData(rpmData, angleData)
        xValue = viewModel.xValue
        if (xValue > 0f) {
            chart.setVisibleXRangeMaximum(AnalyticsViewModel.MAX_POINTS * AnalyticsViewModel.DT)
            chart.moveViewToX(xValue)
        }

        startStopBtn.setOnClickListener {
            paused = !paused
            if (paused) {
                startStopBtn.setIconResource(R.drawable.ic_play)
                startStopBtn.contentDescription = "START"
            } else {
                startStopBtn.setIconResource(R.drawable.ic_pause)
                startStopBtn.contentDescription = getString(R.string.stop)
            }
        }

        // Collect telemetry when fragment is visible
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.updates.collect{
                    if(!paused){
                        updateGraph()
                    }
                }
            }
        }

        if(!paused) updateGraph()
    }

    private fun updateGraph() {
        if (viewModel.rpmEntries.isEmpty()) return

        xValue = viewModel.xValue
        rpmData.values = viewModel.rpmEntries
        angleData.values = viewModel.angleEntries

        chart.data?.notifyDataChanged()
        chart.notifyDataSetChanged()
        chart.setVisibleXRangeMaximum(
            AnalyticsViewModel.MAX_POINTS * AnalyticsViewModel.DT
        )
        chart.moveViewToX(xValue)
    }
}
