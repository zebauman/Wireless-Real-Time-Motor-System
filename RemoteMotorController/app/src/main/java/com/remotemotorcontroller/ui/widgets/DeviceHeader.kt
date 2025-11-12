package com.remotemotorcontroller.ui.widgets

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.widget.TextView
import com.google.android.material.button.MaterialButton
import com.google.android.material.card.MaterialCardView
import com.remotemotorcontroller.R
import com.remotemotorcontroller.ble.BLEManager

class DeviceHeader @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : MaterialCardView(context, attrs, defStyleAttr) {
    private val textConnect: TextView
    private val textSubtitle: TextView

    private val buttonDisconnect: MaterialButton

    // EXTERNAL CLICK LISTENER FOR DISCONNECT BUTTON
    private var onDisconnect: (() -> Unit)? = null

    init {
        LayoutInflater.from(context).inflate(R.layout.include_device_header, this, true)
        textConnect = findViewById(R.id.textConnection)
        textSubtitle = findViewById(R.id.textSubtitle)
        buttonDisconnect = findViewById(R.id.buttonDisconnect)

        buttonDisconnect.setOnClickListener {
            onDisconnect?.invoke() ?: BLEManager.disconnect()
        }

        isClickable = true
        isFocusable = true
    }

    fun setConnectionTitle(text: String){
        textConnect.text = text
    }
    fun setSubtitle(text: String){
        textSubtitle.text = text
    }
    fun setDisconnectVisible(visible: Boolean){
        buttonDisconnect.visibility = if (visible) VISIBLE else GONE
    }
    fun setOnDisconnectClick(listener: (() -> Unit)?){
        onDisconnect = listener
    }
}