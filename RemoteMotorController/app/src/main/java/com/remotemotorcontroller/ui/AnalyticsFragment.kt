package com.remotemotorcontroller.ui

import android.graphics.Color
import android.os.Bundle
import android.view.View
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.MarkerView
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.components.YAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.github.mikephil.charting.formatter.ValueFormatter
import com.github.mikephil.charting.highlight.Highlight
import com.github.mikephil.charting.utils.MPPointF
import com.google.android.material.button.MaterialButton
import com.remotemotorcontroller.R
import com.remotemotorcontroller.adapter.AnalyticsViewModel
import com.remotemotorcontroller.settings.SettingsRepository
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import java.util.Locale

class AnalyticsFragment : Fragment(R.layout.fragment_analytics) {

    private lateinit var chart: LineChart
    private lateinit var startStopBtn: MaterialButton
    private lateinit var resetBtn: MaterialButton

    private lateinit var rpmSet: LineDataSet
    private lateinit var angleSet: LineDataSet

    private lateinit var repo: SettingsRepository

    private var isPaused = false
    private val viewModel: AnalyticsViewModel by activityViewModels()

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        repo = SettingsRepository(requireContext())

        chart = view.findViewById(R.id.chartTelemetry)
        startStopBtn = view.findViewById(R.id.buttonPlayPause)
        resetBtn = view.findViewById(R.id.buttonReset)

        setupChartStyle()
        setupDataSets()
        setupInteractions()
        subscribeToData()

        // Load Max Points from Settings
        viewLifecycleOwner.lifecycleScope.launch {
            val config = repo.settings.first()
            viewModel.applyConfig(config.analy.maxPoints)
        }
    }

    // CHART STYLING
    private fun setupChartStyle() {
        chart.apply {
            description.isEnabled = false
            setTouchEnabled(true)
            isDragEnabled = true
            setScaleEnabled(true)
            setPinchZoom(true)
            setDrawGridBackground(false)
            setBackgroundColor(Color.WHITE)

            // Add a Tooltip when user taps a point
            marker = CustomMarkerView(requireContext(), R.layout.view_chart_marker)
        }

        // X AXIS (Time)
        chart.xAxis.apply {
            position = XAxis.XAxisPosition.BOTTOM
            setDrawGridLines(false)
            textColor = Color.DKGRAY
            textSize = 12f
            // Prevents skipping labels when zoomed out
            isGranularityEnabled = true
            granularity = 1f

            valueFormatter = object : ValueFormatter() {
                override fun getFormattedValue(value: Float): String {
                    val minutes = (value / 60).toInt()
                    val seconds = (value % 60)
                    return String.format(Locale.US, "%02d:%04.1f", minutes, seconds)
                }
            }
        }

        // LEFT Y-AXIS (RPM)
        chart.axisLeft.apply {
            isEnabled = true
            textColor = ContextCompat.getColor(requireContext(), R.color.aggie_maroon)
            setDrawGridLines(true)
            axisMinimum = 0f
        }

        // RIGHT Y-AXIS (ANGLE)
        chart.axisRight.apply {
            isEnabled = true
            textColor = Color.parseColor("#757575")
            setDrawGridLines(false)
            axisMinimum = 0f
            axisMaximum = 360f
        }
    }

    // DATA SETUP
    private fun setupDataSets() {
        // RPM Setup (Aggie Maroon, Left Axis)
        rpmSet = LineDataSet(viewModel.rpmEntries, "RPM").apply {
            color = ContextCompat.getColor(requireContext(), R.color.aggie_maroon)
            lineWidth = 2f
            setDrawCircles(false)
            setDrawValues(false)
            axisDependency = YAxis.AxisDependency.LEFT
            mode = LineDataSet.Mode.CUBIC_BEZIER
        }

        // Angle Setup
        angleSet = LineDataSet(viewModel.angleEntries, "Angle").apply {
            color = Color.parseColor("#757575")
            lineWidth = 1.5f
            setDrawCircles(false)
            setDrawValues(false)
            axisDependency = YAxis.AxisDependency.RIGHT
            enableDashedLine(10f, 5f, 0f)
        }

        chart.data = LineData(rpmSet, angleSet)
    }

    // INTERACTIONS
    private fun setupInteractions() {
        startStopBtn.setOnClickListener {
            isPaused = !isPaused
            if (isPaused) {
                startStopBtn.setIconResource(R.drawable.ic_play)
                startStopBtn.text = "Resume"
            } else {
                startStopBtn.setIconResource(R.drawable.ic_pause)
                startStopBtn.text = "Pause"
                // Snap back to latest data on resume
                if(viewModel.rpmEntries.isNotEmpty()){
                    chart.moveViewToX(viewModel.rpmEntries.last().x)
                }
            }
        }

        resetBtn.setOnClickListener {
            viewModel.reset()
            rpmSet.notifyDataSetChanged()
            angleSet.notifyDataSetChanged()
            chart.data.notifyDataChanged()
            chart.notifyDataSetChanged()
            chart.invalidate()
        }
    }

    // DATA UPDATES
    private fun subscribeToData() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.updates.collect {
                    if (!isPaused && viewModel.rpmEntries.isNotEmpty()) {
                        updateGraph()
                    }
                }
            }
        }
    }

    private fun updateGraph() {
        rpmSet.values = viewModel.rpmEntries
        angleSet.values = viewModel.angleEntries

        chart.data.notifyDataChanged()
        chart.notifyDataSetChanged()

        // Auto-scroll logic
        val lastX = viewModel.rpmEntries.last().x

        // This moves the view to the latest point but allows zooming out
        chart.moveViewToX(lastX)
    }

    // CUSTOM MARKER (Tooltip Class)
    inner class CustomMarkerView(context: android.content.Context, layoutResource: Int) :
        MarkerView(context, layoutResource) {

        private val tvContent: TextView = findViewById(R.id.tvContent)

        override fun refreshContent(e: Entry?, highlight: Highlight?) {
            if (e == null) return

            val value = e.y
            val time = e.x

            tvContent.text = String.format(Locale.US, "T: %.1fs\nVal: %.0f", time, value)

            super.refreshContent(e, highlight)
        }

        override fun getOffset(): MPPointF {
            // Center the marker above the point
            return MPPointF(-(width / 2).toFloat(), -height.toFloat())
        }
    }
}