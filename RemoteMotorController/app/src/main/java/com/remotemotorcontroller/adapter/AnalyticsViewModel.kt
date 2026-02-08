package com.remotemotorcontroller.adapter

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.github.mikephil.charting.data.Entry
import com.remotemotorcontroller.ble.BLEManager
import com.remotemotorcontroller.ble.BleState
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.launch

class AnalyticsViewModel : ViewModel() {

    companion object{
        const val DT = 1f
        var MAX_POINTS = 600
    }
    var xValue = 0f
        private set

    val rpmEntries: MutableList<Entry> = mutableListOf()
    val angleEntries: MutableList<Entry> = mutableListOf()

    private val _updates = MutableSharedFlow<Unit>(extraBufferCapacity = 64)
    val updates = _updates.asSharedFlow()
    init{
        viewModelScope.launch{
            BLEManager.state.collect{state ->

                if(state is BleState.Connected){
                    if(state.telemetry != null){
                        appendPoint(state.telemetry.rpm, state.telemetry.angle)
                    }
                }
            }
        }
    }

    fun appendPoint(rpm: Int, angle: Int){
        rpmEntries.add(Entry(xValue, rpm.toFloat()))
        angleEntries.add(Entry(xValue, angle.toFloat()))

        if(rpmEntries.size > MAX_POINTS) rpmEntries.removeAt(0)
        if(angleEntries.size > MAX_POINTS) angleEntries.removeAt(0)
        xValue += DT

        _updates.tryEmit(Unit)  // NOTIFY THE ANALYTIC UI THAT DATA HAS CHANGED
    }

    fun reset(){
        xValue = 0f
        rpmEntries.clear()
        angleEntries.clear()
        _updates.tryEmit(Unit)
    }

    fun applyConfig(maxPts: Int){
        MAX_POINTS = maxPts
        _updates.tryEmit(Unit)
    }
}