package com.nixus.rccar

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.drawable.GradientDrawable
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.view.Gravity
import android.view.MotionEvent
import android.view.Surface
import android.view.View
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan2

private val BG = 0xFF0E1116.toInt()
private val CARD = 0xFF171B22.toInt()
private val BORDER = 0xFF2A303B.toInt()
private val TEXT = 0xFFE6EAF0.toInt()
private val MUTED = 0xFF8A93A3.toInt()
private val ACCENT = 0xFF3B82F6.toInt()
private val GOOD = 0xFF22C55E.toInt()
private val WARN = 0xFFF59E0B.toInt()
private val BAD = 0xFFEF4444.toInt()

private const val FLAG_LINK_LOST = 1
private const val FLAG_LIMP = 2
private const val FLAG_CUTOFF = 4
private const val FLAG_FLIPPED = 8
private const val FLAG_BLOCKED = 16
private const val FLAG_CRASH = 32
private const val FLAG_HOLD = 64
private const val FLAG_STAB = 128
private const val FLAG_HEADING = 256

private const val REQ_BT = 1
private const val TILT_MAX_DEG = 40f
private const val TILT_DEADZONE_DEG = 3f

class MainActivity : Activity(), CarLink.Listener, SensorEventListener {

    private val ui = Handler(Looper.getMainLooper())
    private val link = CarLink(this)

    @Volatile private var latest = Telemetry()
    @Volatile private var lastTelemetryAt = 0L
    private var linkState = CarLink.State.DISCONNECTED
    private var linkMessage = "Not connected"

    private var curThrottle = 0
    private var curSteer = 0
    private var lightsOn = false
    private var hazardOn = false
    private var holdArmed = false
    private var tiltOn = false
    private var tiltReverse = false
    private var tiltFiltered = 0f
    private var pendingAction: (() -> Unit)? = null

    private lateinit var throttlePad: AxisPad
    private lateinit var steerPad: AxisPad
    private lateinit var statusText: TextView
    private lateinit var connectChip: TextView
    private lateinit var alertText: TextView
    private lateinit var batteryM: Metric
    private lateinit var speedM: Metric
    private lateinit var tiltM: Metric
    private lateinit var sonarC: Metric
    private lateinit var sonarL: Metric
    private lateinit var sonarR: Metric
    private lateinit var modeChips: List<TextView>
    private lateinit var autoChips: List<TextView>
    private lateinit var lightsChip: TextView
    private lateinit var hazardChip: TextView
    private lateinit var holdChip: TextView
    private lateinit var stabChip: TextView
    private lateinit var headingChip: TextView
    private lateinit var tiltChip: TextView
    private lateinit var tiltRevChip: TextView
    private lateinit var rampLabel: TextView
    private lateinit var rampSeek: SeekBar
    private lateinit var brightSeek: SeekBar
    private lateinit var sensors: SensorManager

    private class Metric(val root: LinearLayout, val value: TextView)

    // ------------------------------------------------------------------ lifecycle

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        if (Build.VERSION.SDK_INT >= 28) {
            val attrs = window.attributes
            attrs.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
            window.attributes = attrs
        }
        sensors = getSystemService(SENSOR_SERVICE) as SensorManager
        buildUi()
        ui.postDelayed({ autoConnect() }, 400)
    }

    override fun onResume() {
        super.onResume()
        ui.post(renderTask)
        if (tiltOn) registerTilt()
    }

    override fun onPause() {
        ui.removeCallbacks(renderTask)
        sensors.unregisterListener(this)
        curThrottle = 0
        throttlePad.setExternal(0)
        link.setDrive(0, if (tiltOn) 0 else curSteer)
        link.emergencyStopNow()
        super.onPause()
    }

    override fun onDestroy() {
        link.disconnect()
        super.onDestroy()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) hideSystemUi()
    }

    @Suppress("DEPRECATION")
    private fun hideSystemUi() {
        window.decorView.systemUiVisibility = (View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                or View.SYSTEM_UI_FLAG_FULLSCREEN
                or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION)
    }

    // ------------------------------------------------------------------ small UI helpers

    private fun dp(v: Int): Int = (v * resources.displayMetrics.density).toInt()

    private fun styleChip(tv: TextView, on: Boolean, color: Int = ACCENT) {
        val g = GradientDrawable()
        g.cornerRadius = dp(10).toFloat()
        g.setColor(if (on) color else CARD)
        g.setStroke(dp(1), if (on) color else BORDER)
        tv.background = g
        tv.setTextColor(if (on && color == WARN) 0xFF1A1200.toInt() else TEXT)
    }

    private fun chip(label: String, onClick: () -> Unit): TextView {
        val tv = TextView(this)
        tv.text = label
        tv.textSize = 13f
        tv.gravity = Gravity.CENTER
        tv.minHeight = dp(42)
        tv.setPadding(dp(6), dp(6), dp(6), dp(6))
        tv.setOnClickListener { onClick() }
        styleChip(tv, false)
        return tv
    }

    private fun metric(title: String): Metric {
        val box = LinearLayout(this)
        box.orientation = LinearLayout.VERTICAL
        box.setPadding(dp(10), dp(8), dp(10), dp(8))
        val g = GradientDrawable()
        g.cornerRadius = dp(12).toFloat()
        g.setColor(CARD)
        g.setStroke(dp(1), BORDER)
        box.background = g
        val t = TextView(this)
        t.text = title
        t.setTextColor(MUTED)
        t.textSize = 11f
        val v = TextView(this)
        v.text = "-"
        v.setTextColor(TEXT)
        v.textSize = 17f
        box.addView(t)
        box.addView(v)
        return Metric(box, v)
    }

    private fun row(vararg views: View): LinearLayout {
        val r = LinearLayout(this)
        r.orientation = LinearLayout.HORIZONTAL
        for (v in views) {
            val lp = LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f)
            lp.setMargins(dp(3), dp(3), dp(3), dp(3))
            r.addView(v, lp)
        }
        return r
    }

    private fun label(text: String): TextView {
        val t = TextView(this)
        t.text = text
        t.setTextColor(MUTED)
        t.textSize = 12f
        t.setPadding(dp(4), dp(10), dp(4), dp(2))
        return t
    }

    private fun seekColors(s: SeekBar) {
        s.progressTintList = ColorStateList.valueOf(ACCENT)
        s.thumbTintList = ColorStateList.valueOf(ACCENT)
    }

    // ------------------------------------------------------------------ UI

    private fun buildUi() {
        throttlePad = AxisPad(this, true)
        throttlePad.accent = GOOD
        throttlePad.onValue = { v ->
            curThrottle = v
            pushDrive()
        }
        throttlePad.onGrab = { down ->
            // touching the throttle again cancels a held speed
            if (down && (latest.flags and FLAG_HOLD) != 0) link.sendQueued("K,0")
        }
        throttlePad.onRelease = { last ->
            if (holdArmed && last > 0 && latest.auto == 0) {
                // lock the speed the car is doing right now, then let go of the pad
                link.sendQueued("D,$last,$curSteer")
                link.sendQueued("K,1")
            }
        }

        steerPad = AxisPad(this, false)
        steerPad.accent = ACCENT
        steerPad.onValue = { v ->
            if (!tiltOn) {
                curSteer = v
                pushDrive()
            }
        }

        // ---- status row
        statusText = TextView(this)
        statusText.setTextColor(TEXT)
        statusText.textSize = 14f
        statusText.gravity = Gravity.CENTER_VERTICAL
        connectChip = chip("Connect") { onConnectClicked() }
        val statusRow = LinearLayout(this)
        statusRow.orientation = LinearLayout.HORIZONTAL
        statusRow.gravity = Gravity.CENTER_VERTICAL
        statusRow.addView(statusText, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        statusRow.addView(connectChip, LinearLayout.LayoutParams(dp(110), ViewGroup.LayoutParams.WRAP_CONTENT))

        // ---- metrics
        batteryM = metric("Battery")
        speedM = metric("Speed")
        tiltM = metric("Tilt")
        sonarC = metric("Front")
        sonarL = metric("Left")
        sonarR = metric("Right")

        alertText = TextView(this)
        alertText.textSize = 13f
        alertText.setPadding(dp(8), dp(8), dp(8), dp(8))

        // ---- mode and auto
        modeChips = listOf("Eco", "Normal", "Sport").mapIndexed { i, n ->
            chip(n) {
                link.sendQueued("M,$i")
                val defaults = intArrayOf(600, 300, 150)
                rampSeek.progress = defaults[i] / 50
            }
        }
        autoChips = listOf("Manual", "Roam", "Follow", "Corridor").mapIndexed { i, n ->
            chip(n) { link.sendQueued("A,$i") }
        }

        // ---- toggles
        lightsChip = chip("Lights") {
            lightsOn = !lightsOn
            link.sendQueued("L,${if (lightsOn) 1 else 0}")
            styleChip(lightsChip, lightsOn)
        }
        hazardChip = chip("Hazard") {
            hazardOn = !hazardOn
            link.sendQueued("H,${if (hazardOn) 1 else 0}")
            styleChip(hazardChip, hazardOn, WARN)
        }
        val hornChip = chip("Horn (hold)") { }
        hornChip.setOnTouchListener { _, e ->
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    link.sendQueued("V,1")
                    styleChip(hornChip, true, WARN)
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    link.sendQueued("V,0")
                    styleChip(hornChip, false)
                }
            }
            true
        }
        val brakeChip = chip("BRAKE") { link.sendQueued("X") }
        styleChip(brakeChip, true, BAD)

        holdChip = chip("Hold speed on release") {
            holdArmed = !holdArmed
            if (!holdArmed && (latest.flags and FLAG_HOLD) != 0) link.sendQueued("K,0")
            styleChip(holdChip, holdArmed)
        }
        stabChip = chip("Stability assist") {
            val on = (latest.flags and FLAG_STAB) != 0
            link.sendQueued("S,${if (on) 0 else 1}")
        }
        headingChip = chip("Heading hold") {
            val on = (latest.flags and FLAG_HEADING) != 0
            link.sendQueued("G,${if (on) 0 else 1}")
        }
        tiltChip = chip("Tilt steering") { toggleTilt() }
        tiltRevChip = chip("Reverse tilt") {
            tiltReverse = !tiltReverse
            styleChip(tiltRevChip, tiltReverse)
        }
        val calChip = chip("Recalibrate gyro") {
            link.sendQueued("Z")
            toast("Keep the car still for a second")
        }

        // ---- sliders
        brightSeek = SeekBar(this)
        brightSeek.max = 100
        brightSeek.progress = 100
        seekColors(brightSeek)
        brightSeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) {
                if (fromUser) link.sendLatest("B", "B,$p")
            }
            override fun onStartTrackingTouch(s: SeekBar) {}
            override fun onStopTrackingTouch(s: SeekBar) {
                link.sendLatest("B", "B,${s.progress}")
            }
        })
        rampLabel = label("Acceleration ramp: 300 ms")
        rampSeek = SeekBar(this)
        rampSeek.max = 30
        rampSeek.progress = 6
        seekColors(rampSeek)
        rampSeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) {
                rampLabel.text = "Acceleration ramp: ${p * 50} ms" + if (p == 0) " (launch mode)" else ""
                if (fromUser) link.sendLatest("R", "R,${p * 50}")
            }
            override fun onStartTrackingTouch(s: SeekBar) {}
            override fun onStopTrackingTouch(s: SeekBar) {
                link.sendLatest("R", "R,${s.progress * 50}")
            }
        })

        // ---- center column
        val center = LinearLayout(this)
        center.orientation = LinearLayout.VERTICAL
        center.setPadding(dp(8), dp(4), dp(8), dp(16))
        center.addView(statusRow)
        center.addView(row(batteryM.root, speedM.root, tiltM.root))
        center.addView(row(sonarL.root, sonarC.root, sonarR.root))
        center.addView(alertText)
        center.addView(label("Drive mode"))
        center.addView(row(modeChips[0], modeChips[1], modeChips[2]))
        center.addView(label("Autonomous"))
        center.addView(row(autoChips[0], autoChips[1]))
        center.addView(row(autoChips[2], autoChips[3]))
        center.addView(label("Controls"))
        center.addView(row(lightsChip, hazardChip, hornChip))
        center.addView(row(brakeChip, holdChip))
        center.addView(label("Assists"))
        center.addView(row(stabChip, headingChip))
        center.addView(row(tiltChip, tiltRevChip))
        center.addView(label("Headlight brightness"))
        center.addView(brightSeek)
        center.addView(rampLabel)
        center.addView(rampSeek)
        center.addView(row(calChip))

        val scroll = ScrollView(this)
        scroll.isVerticalScrollBarEnabled = false
        scroll.addView(center)

        // ---- root: throttle | center | steering
        val left = FrameLayout(this)
        left.addView(throttlePad, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT).also {
            it.setMargins(dp(4), dp(4), dp(4), dp(4))
        })
        val right = FrameLayout(this)
        right.addView(steerPad, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, dp(150), Gravity.BOTTOM).also {
            it.setMargins(dp(4), dp(4), dp(4), dp(4))
        })

        val root = LinearLayout(this)
        root.orientation = LinearLayout.HORIZONTAL
        root.setBackgroundColor(BG)
        root.setPadding(dp(28), dp(8), dp(28), dp(8))
        root.addView(left, LinearLayout.LayoutParams(dp(120), ViewGroup.LayoutParams.MATCH_PARENT))
        root.addView(scroll, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 1f))
        root.addView(right, LinearLayout.LayoutParams(dp(330), ViewGroup.LayoutParams.MATCH_PARENT))
        setContentView(root)
    }

    private fun toast(msg: String) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
    }

    private fun pushDrive() {
        link.setDrive(curThrottle, curSteer)
    }

    // ------------------------------------------------------------------ rendering (10 times a second)

    private val renderTask = object : Runnable {
        override fun run() {
            render()
            ui.postDelayed(this, 100)
        }
    }

    private fun distText(d: Int): String = if (d >= 900) "clear" else "$d cm"

    private fun distColor(d: Int): Int = when {
        d < 20 -> BAD
        d < 40 -> WARN
        else -> TEXT
    }

    private fun render() {
        val t = latest
        val connected = linkState == CarLink.State.CONNECTED
        val fresh = connected && SystemClock.elapsedRealtime() - lastTelemetryAt < 1500

        statusText.text = when {
            fresh -> "Connected: $linkMessage"
            connected -> "Connected, waiting for the car..."
            linkState == CarLink.State.CONNECTING -> "Connecting to $linkMessage..."
            else -> linkMessage
        }
        statusText.setTextColor(if (fresh) GOOD else if (connected) WARN else MUTED)
        connectChip.text = if (connected) "Disconnect" else "Connect"

        if (fresh) {
            if (t.battV < 3f) {
                batteryM.value.text = "USB only"
                batteryM.value.setTextColor(MUTED)
            } else {
                batteryM.value.text = String.format(Locale.US, "%.2f V", t.battV)
                batteryM.value.setTextColor(if (t.battV < 6.8f) BAD else if (t.battV < 7.2f) WARN else GOOD)
            }
            speedM.value.text = "${t.speed}%"
            tiltM.value.text = "R${t.roll} P${t.pitch}"
            sonarC.value.text = distText(t.distC)
            sonarC.value.setTextColor(distColor(t.distC))
            sonarL.value.text = distText(t.distL)
            sonarL.value.setTextColor(distColor(t.distL))
            sonarR.value.text = distText(t.distR)
            sonarR.value.setTextColor(distColor(t.distR))
        } else {
            for (m in listOf(batteryM, speedM, tiltM, sonarC, sonarL, sonarR)) {
                m.value.text = "-"
                m.value.setTextColor(MUTED)
            }
        }

        // alerts
        val f = t.flags
        val parts = ArrayList<String>()
        if (fresh) {
            if (f and FLAG_LINK_LOST != 0) parts.add("Link lost, car stopped")
            if (f and FLAG_CUTOFF != 0) parts.add("Battery cutoff")
            if (f and FLAG_FLIPPED != 0) parts.add("Flipped")
            if (f and FLAG_CRASH != 0) parts.add("Crash detected")
            if (f and FLAG_LIMP != 0) parts.add("Limp mode: battery low")
            if (f and FLAG_BLOCKED != 0) parts.add("Obstacle ahead")
            if (f and FLAG_HOLD != 0) parts.add("Holding speed (touch throttle to release)")
        }
        val danger = f and (FLAG_LINK_LOST or FLAG_CUTOFF or FLAG_FLIPPED or FLAG_CRASH) != 0
        val warn = f and (FLAG_LIMP or FLAG_BLOCKED) != 0
        alertText.text = if (!fresh) "" else if (parts.isEmpty()) "All clear" else parts.joinToString("  |  ")
        alertText.setTextColor(if (!fresh) MUTED else if (danger) BAD else if (warn) WARN else if (parts.isEmpty()) GOOD else ACCENT)

        // chip highlights follow what the car reports
        for (i in modeChips.indices) styleChip(modeChips[i], i == t.mode)
        for (i in autoChips.indices) styleChip(autoChips[i], i == t.auto)
        styleChip(stabChip, f and FLAG_STAB != 0)
        styleChip(headingChip, f and FLAG_HEADING != 0)
        val holding = f and FLAG_HOLD != 0
        styleChip(holdChip, holdArmed || holding, if (holding) WARN else ACCENT)
        styleChip(tiltChip, tiltOn)
    }

    // ------------------------------------------------------------------ CarLink.Listener

    override fun onState(state: CarLink.State, message: String) {
        ui.post {
            linkState = state
            linkMessage = message
            if (state == CarLink.State.CONNECTED) syncSettings()
        }
    }

    override fun onTelemetry(t: Telemetry) {
        latest = t
        lastTelemetryAt = SystemClock.elapsedRealtime()
    }

    /** Push the app's current settings to the car right after connecting. */
    private fun syncSettings() {
        val mode = 1
        link.sendQueued("M,$mode")
        link.sendQueued("A,0")
        link.sendQueued("L,${if (lightsOn) 1 else 0}")
        link.sendLatest("B", "B,${brightSeek.progress}")
        link.sendLatest("R", "R,${rampSeek.progress * 50}")
        pushDrive()
    }

    // ------------------------------------------------------------------ Bluetooth connect

    private fun onConnectClicked() {
        if (link.isConnected || linkState == CarLink.State.CONNECTING) {
            link.disconnect()
            linkState = CarLink.State.DISCONNECTED
            linkMessage = "Not connected"
        } else {
            pickDevice()
        }
    }

    private fun adapter(): BluetoothAdapter? {
        val m = getSystemService(BluetoothManager::class.java)
        return m?.adapter
    }

    private fun withBluetoothPermission(action: () -> Unit) {
        if (Build.VERSION.SDK_INT >= 31 &&
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED
        ) {
            pendingAction = action
            requestPermissions(arrayOf(Manifest.permission.BLUETOOTH_CONNECT), REQ_BT)
            return
        }
        action()
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQ_BT) {
            val ok = grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }
            val action = pendingAction
            pendingAction = null
            if (ok && action != null) action() else toast("Bluetooth permission is needed to reach the car")
        }
    }

    @SuppressLint("MissingPermission")
    private fun pickDevice() {
        withBluetoothPermission {
            val ad = adapter()
            if (ad == null) {
                toast("This phone has no Bluetooth")
            } else if (!ad.isEnabled) {
                toast("Turn Bluetooth on first")
            } else {
                val devices = ad.bondedDevices.toList().sortedBy { it.name ?: "" }
                if (devices.isEmpty()) {
                    toast("Pair the car's Bluetooth module in Android settings first (PIN 1234)")
                } else {
                    val names = devices.map { (it.name ?: "Unknown") + "   " + it.address }.toTypedArray()
                    AlertDialog.Builder(this, android.R.style.Theme_Material_Dialog_Alert)
                        .setTitle("Choose your car")
                        .setItems(names) { _, i -> connectTo(devices[i]) }
                        .show()
                }
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun connectTo(device: BluetoothDevice) {
        getSharedPreferences("rc", MODE_PRIVATE).edit().putString("last", device.address).apply()
        adapter()?.cancelDiscovery()
        link.connect(device)
    }

    @SuppressLint("MissingPermission")
    private fun autoConnect() {
        val addr = getSharedPreferences("rc", MODE_PRIVATE).getString("last", null) ?: return
        if (link.isConnected) return
        withBluetoothPermission {
            val ad = adapter()
            if (ad != null && ad.isEnabled) {
                try {
                    connectTo(ad.getRemoteDevice(addr))
                } catch (e: IllegalArgumentException) {
                    // saved address no longer valid
                }
            }
        }
    }

    // ------------------------------------------------------------------ tilt steering

    private fun toggleTilt() {
        tiltOn = !tiltOn
        steerPad.interactive = !tiltOn
        if (tiltOn) {
            tiltFiltered = 0f
            registerTilt()
            toast("Hold the phone upright like a steering wheel")
        } else {
            sensors.unregisterListener(this)
            curSteer = 0
            steerPad.setExternal(0)
            pushDrive()
        }
        styleChip(tiltChip, tiltOn)
    }

    private fun registerTilt() {
        val acc = sensors.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        if (acc == null) {
            toast("No accelerometer on this phone")
            tiltOn = false
            steerPad.interactive = true
            return
        }
        sensors.registerListener(this, acc, SensorManager.SENSOR_DELAY_GAME)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    @Suppress("DEPRECATION")
    override fun onSensorChanged(ev: SensorEvent) {
        if (!tiltOn) return
        val ax = ev.values[0]
        val ay = ev.values[1]
        // express gravity in screen coordinates (x to the right, y up) whatever way the phone is turned
        val sx: Float
        val sy: Float
        when (windowManager.defaultDisplay.rotation) {
            Surface.ROTATION_90 -> { sx = -ay; sy = ax }
            Surface.ROTATION_270 -> { sx = ay; sy = -ax }
            Surface.ROTATION_180 -> { sx = -ax; sy = -ay }
            else -> { sx = ax; sy = ay }
        }
        // turning the phone clockwise like a wheel gives a negative angle, so steer right = -angle
        val angle = Math.toDegrees(atan2(sx.toDouble(), sy.toDouble())).toFloat()
        tiltFiltered += 0.3f * (-angle - tiltFiltered)
        var d = tiltFiltered
        d = if (abs(d) < TILT_DEADZONE_DEG) 0f else (d - Math.signum(d) * TILT_DEADZONE_DEG) / (TILT_MAX_DEG - TILT_DEADZONE_DEG) * 100f
        if (tiltReverse) d = -d
        val s = d.coerceIn(-100f, 100f).toInt()
        if (s != curSteer) {
            curSteer = s
            steerPad.setExternal(s)
            pushDrive()
        }
    }
}
