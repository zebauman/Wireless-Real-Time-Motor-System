package com.remotemotorcontroller.ui.widgets

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.widget.FrameLayout
import android.widget.TextView
import com.google.android.material.card.MaterialCardView
import com.google.android.material.chip.Chip
import com.remotemotorcontroller.R

class LiveSummaryView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : FrameLayout(context, attrs, defStyleAttr) {

    private val textRpm: TextView
    private val textAngle: TextView

    init {
        LayoutInflater.from(context).inflate(R.layout.include_live_summary, this, true)

        textRpm = findViewById(R.id.textRpm)
        textAngle = findViewById(R.id.textAngle)
    }

    fun setRpm(rpm: Int) {
        textRpm.text = context.getString(R.string.label_rpm, rpm)
    }

    fun setAngle(angle: Int) {
        textAngle.text = context.getString(R.string.label_angle, angle)
    }
}
