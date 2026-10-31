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

private const val STEER_W_DP = 330
private const val STEER_H_DP = 150
private const val THROTTLE_W_DP = 120
private const val BOTTOM_ROW_DP = 52
private const val DEFAULT_PAD_LEFT = 40
private const val DEFAULT_STEER_RAISE = 8
private val MODE_NAMES = arrayOf("Eco", "Normal", "Sport")
private val AUTO_NAMES = arrayOf("Manual", "Roam", "Follow", "Corridor")

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
    private lateinit var sonarView: SonarView
    private lateinit var statusText: TextView
    private lateinit var connectChip: TextView
    private lateinit var alertText: TextView
    private lateinit var calInfo: TextView
    private lateinit var batteryM: Metric
    private lateinit var speedM: Metric
    private lateinit var tiltM: Metric
    private lateinit var modeChips: List<TextView>
    private lateinit var autoChips: List<TextView>
    private lateinit var modesBtn: TextView
    private lateinit var autoBtn: TextView
    private lateinit var assistBtn: TextView
    private lateinit var settingsBtn: TextView
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
    private lateinit var padSeek: SeekBar
    private lateinit var raiseSeek: SeekBar
    private lateinit var sensors: SensorManager

    // layout pieces whose margins the position sliders move
    private lateinit var throttleLp: FrameLayout.LayoutParams
    private lateinit var steerLp: FrameLayout.LayoutParams
    private lateinit var centerLp: FrameLayout.LayoutParams
    private lateinit var centerBox: LinearLayout
    private var padLeftDp = DEFAULT_PAD_LEFT
    private var steerRaiseDp = DEFAULT_STEER_RAISE
    private var popupDialog: AlertDialog? = null

    private class Metric(val root: LinearLayout, val value: TextView)

    // ------------------------------------------------------------------ lifecycle

    /** Saves the stack trace of a crash so the next launch can show it on screen. */
    private fun installCrashCatcher() {
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { t, e ->
            try {
                val sw = java.io.StringWriter()
                e.printStackTrace(java.io.PrintWriter(sw))
                getSharedPreferences("rc", MODE_PRIVATE).edit().putString("crash", sw.toString()).commit()
            } catch (ignored: Exception) {
            }
            previous?.uncaughtException(t, e)
        }
    }

    private fun showLastCrash() {
        val prefs = getSharedPreferences("rc", MODE_PRIVATE)
        val trace = prefs.getString("crash", null) ?: return
        prefs.edit().remove("crash").apply()
        AlertDialog.Builder(this, android.R.style.Theme_Material_Dialog_Alert)
            .setTitle("The app crashed last time")
            .setMessage(trace.take(1800))
            .setPositiveButton("Close") { _, _ -> }
            .show()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        installCrashCatcher()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        if (Build.VERSION.SDK_INT >= 28) {
            val attrs = window.attributes
            attrs.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
            window.attributes = attrs
        }
        sensors = getSystemService(SENSOR_SERVICE) as SensorManager
        buildUi()
        showLastCrash()
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

    private fun chip(label: String, size: Float = 13f, onClick: () -> Unit): TextView {
        val tv = TextView(this)
        tv.text = label
        tv.textSize = size
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
        box.setPadding(dp(8), dp(5), dp(8), dp(5))
        val g = GradientDrawable()
        g.cornerRadius = dp(12).toFloat()
        g.setColor(CARD)
        g.setStroke(dp(1), BORDER)
        box.background = g
        val t = TextView(this)
        t.text = title
        t.setTextColor(MUTED)
        t.textSize = 10f
        val v = TextView(this)
        v.text = "-"
        v.setTextColor(TEXT)
        v.textSize = 15f
        box.addView(t)
        box.addView(v)
        return Metric(box, v)
    }

    /** A view can only have one parent, so take it out of the old one (for example an earlier popup) first. */
    private fun <T : View> detach(v: T): T {
        (v.parent as? ViewGroup)?.removeView(v)
        return v
    }

    private fun row(vararg views: View): LinearLayout {
        val r = LinearLayout(this)
        r.orientation = LinearLayout.HORIZONTAL
        for (v in views) {
            detach(v)
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

    private fun note(text: String): TextView {
        val t = TextView(this)
        t.text = text
        t.setTextColor(MUTED)
        t.textSize = 12f
        t.setPadding(dp(4), dp(8), dp(4), dp(4))
        return t
    }

    private fun seekColors(s: SeekBar) {
        s.progressTintList = ColorStateList.valueOf(ACCENT)
        s.thumbTintList = ColorStateList.valueOf(ACCENT)
    }

    private fun column(): LinearLayout {
        val c = LinearLayout(this)
        c.orientation = LinearLayout.VERTICAL
        c.setPadding(dp(14), dp(6), dp(14), dp(6))
        return c
    }

    private fun popup(title: String, content: LinearLayout) {
        popupDialog?.dismiss()
        val sv = ScrollView(this)
        sv.isVerticalScrollBarEnabled = false
        sv.addView(content)
        popupDialog = AlertDialog.Builder(this, android.R.style.Theme_Material_Dialog_Alert)
            .setTitle(title)
            .setView(sv)
            .setPositiveButton("Done") { _, _ -> }
            .show()
    }

    private fun closePopup() {
        popupDialog?.dismiss()
        popupDialog = null
    }

    private fun requireCar(action: () -> Unit) {
        if (linkState != CarLink.State.CONNECTED) toast("Connect to the car first") else action()
    }

    private fun prefs() = getSharedPreferences("rc", MODE_PRIVATE)

    // ------------------------------------------------------------------ UI

    private fun buildUi() {
        padLeftDp = prefs().getInt("padLeft", DEFAULT_PAD_LEFT).coerceIn(0, 120)
        steerRaiseDp = prefs().getInt("steerRaise", DEFAULT_STEER_RAISE).coerceIn(0, 100)

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

        // ---- status
        statusText = TextView(this)
        statusText.setTextColor(TEXT)
        statusText.textSize = 13f
        statusText.gravity = Gravity.CENTER_VERTICAL
        connectChip = chip("Connect", 12f) { onConnectClicked() }
        val statusRow = LinearLayout(this)
        statusRow.orientation = LinearLayout.HORIZONTAL
        statusRow.gravity = Gravity.CENTER_VERTICAL
        val cl = LinearLayout.LayoutParams(dp(100), ViewGroup.LayoutParams.WRAP_CONTENT)
        cl.setMargins(0, 0, dp(8), 0)
        statusRow.addView(connectChip, cl)
        statusRow.addView(statusText, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))

        batteryM = metric("Battery")
        speedM = metric("Speed")
        tiltM = metric("Tilt")
        sonarView = SonarView(this)

        alertText = TextView(this)
        alertText.textSize = 12f
        alertText.gravity = Gravity.CENTER_VERTICAL
        alertText.minHeight = dp(34)
        alertText.setPadding(dp(6), dp(2), dp(6), dp(2))

        calInfo = TextView(this)
        calInfo.setTextColor(TEXT)
        calInfo.textSize = 16f
        calInfo.setPadding(dp(4), dp(10), dp(4), dp(10))

        // ---- mode and auto (live inside popups)
        modeChips = MODE_NAMES.mapIndexed { i, n ->
            chip(n) {
                link.sendQueued("M,$i")
                val defaults = intArrayOf(600, 300, 150)
                rampSeek.progress = defaults[i] / 50
                closePopup()
            }
        }
        autoChips = AUTO_NAMES.mapIndexed { i, n ->
            chip(n) {
                link.sendQueued("A,$i")
                closePopup()
            }
        }

        // ---- always-visible toggles
        lightsChip = chip("Lights", 12f) {
            lightsOn = !lightsOn
            link.sendQueued("L,${if (lightsOn) 1 else 0}")
            styleChip(lightsChip, lightsOn)
        }
        hazardChip = chip("Hazard", 12f) {
            hazardOn = !hazardOn
            link.sendQueued("H,${if (hazardOn) 1 else 0}")
            styleChip(hazardChip, hazardOn, WARN)
        }
        holdChip = chip("Hold speed", 12f) {
            holdArmed = !holdArmed
            if (!holdArmed && (latest.flags and FLAG_HOLD) != 0) link.sendQueued("K,0")
            styleChip(holdChip, holdArmed)
        }
        tiltChip = chip("Tilt steer", 12f) { toggleTilt() }

        // ---- horn and brake sit under the steering pad
        val hornChip = chip("Horn (hold)") { }
        hornChip.minHeight = dp(46)
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
        brakeChip.minHeight = dp(46)
        styleChip(brakeChip, true, BAD)

        // ---- popup chips
        stabChip = chip("Stability assist") {
            val on = (latest.flags and FLAG_STAB) != 0
            link.sendQueued("S,${if (on) 0 else 1}")
        }
        headingChip = chip("Heading hold") {
            val on = (latest.flags and FLAG_HEADING) != 0
            link.sendQueued("G,${if (on) 0 else 1}")
        }
        tiltRevChip = chip("Reverse tilt") {
            tiltReverse = !tiltReverse
            styleChip(tiltRevChip, tiltReverse)
        }

        // ---- buttons that open popups (right column, above the steering pad)
        modesBtn = chip("Mode") { showModes() }
        autoBtn = chip("Auto") { showAuto() }
        assistBtn = chip("Assists") { showAssists() }
        settingsBtn = chip("Settings") { showSettings() }

        // ---- sliders (live inside the Settings popup)
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
        padSeek = SeekBar(this)
        padSeek.max = 120
        padSeek.progress = padLeftDp
        seekColors(padSeek)
        padSeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) {
                if (fromUser) {
                    padLeftDp = p
                    applyPadPositions()
                }
            }
            override fun onStartTrackingTouch(s: SeekBar) {}
            override fun onStopTrackingTouch(s: SeekBar) {
                prefs().edit().putInt("padLeft", padLeftDp).apply()
            }
        })
        raiseSeek = SeekBar(this)
        raiseSeek.max = 100
        raiseSeek.progress = steerRaiseDp
        seekColors(raiseSeek)
        raiseSeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) {
                if (fromUser) {
                    steerRaiseDp = p
                    applyPadPositions()
                }
            }
            override fun onStartTrackingTouch(s: SeekBar) {}
            override fun onStopTrackingTouch(s: SeekBar) {
                prefs().edit().putInt("steerRaise", steerRaiseDp).apply()
            }
        })

        // ---- center column: status, readings, sonar picture, alerts, toggles
        centerBox = LinearLayout(this)
        centerBox.orientation = LinearLayout.VERTICAL
        centerBox.addView(statusRow)
        centerBox.addView(row(batteryM.root, speedM.root, tiltM.root))
        val sonarLp = LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f)
        sonarLp.setMargins(dp(3), dp(3), dp(3), dp(3))
        centerBox.addView(sonarView, sonarLp)
        centerBox.addView(alertText)
        centerBox.addView(row(lightsChip, hazardChip, holdChip, tiltChip))

        // ---- right column: popup buttons on top, steering pad, horn and brake at the bottom
        val rightBox = FrameLayout(this)
        val topGrid = LinearLayout(this)
        topGrid.orientation = LinearLayout.VERTICAL
        topGrid.addView(row(modesBtn, autoBtn))
        topGrid.addView(row(assistBtn, settingsBtn))
        rightBox.addView(topGrid, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT, Gravity.TOP))
        steerLp = FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, dp(STEER_H_DP), Gravity.BOTTOM)
        rightBox.addView(steerPad, steerLp)
        rightBox.addView(row(hornChip, brakeChip), FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT, Gravity.BOTTOM))

        // ---- root: everything is placed by margins so the pad positions can be nudged
        val root = FrameLayout(this)
        root.setBackgroundColor(BG)
        root.setPadding(dp(28), dp(8), dp(28), dp(8))
        throttleLp = FrameLayout.LayoutParams(dp(THROTTLE_W_DP), ViewGroup.LayoutParams.MATCH_PARENT, Gravity.LEFT)
        root.addView(throttlePad, throttleLp)
        centerLp = FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT, Gravity.LEFT)
        root.addView(centerBox, centerLp)
        root.addView(rightBox, FrameLayout.LayoutParams(dp(STEER_W_DP), ViewGroup.LayoutParams.MATCH_PARENT, Gravity.RIGHT))
        applyPadPositions()
        setContentView(root)
    }

    /** Throttle sits padLeft in from the edge; the steering pad is lifted above the horn and brake row. */
    private fun applyPadPositions() {
        throttleLp.setMargins(dp(padLeftDp), dp(4), dp(4), dp(4))
        throttlePad.layoutParams = throttleLp
        centerLp.setMargins(dp(padLeftDp + THROTTLE_W_DP + 10), 0, dp(STEER_W_DP + 10), 0)
        centerBox.layoutParams = centerLp
        steerLp.setMargins(dp(4), 0, dp(4), dp(BOTTOM_ROW_DP + steerRaiseDp))
        steerPad.layoutParams = steerLp
    }

    // ------------------------------------------------------------------ popups

    private fun showModes() {
        val c = column()
        c.addView(label("Drive mode"))
        c.addView(row(modeChips[0], modeChips[1], modeChips[2]))
        c.addView(note("Top speed: Eco 55%, Normal 80%, Sport 100%. Picking a mode also sets a matching acceleration ramp."))
        popup("Drive mode", c)
    }

    private fun showAuto() {
        val c = column()
        c.addView(row(autoChips[0], autoChips[1]))
        c.addView(row(autoChips[2], autoChips[3]))
        c.addView(note("Roam: drives around and steers away from walls. Follow: follows something in front of it. Corridor: keeps to the middle between two walls. Any of them stops if the link drops."))
        popup("Autonomous", c)
    }

    private fun showAssists() {
        val c = column()
        c.addView(row(stabChip, headingChip))
        c.addView(row(tiltRevChip))
        c.addView(note("Stability assist calms the car when it starts to spin. Heading hold keeps it straight when you steer to center. Reverse tilt swaps the direction of tilt steering."))
        popup("Assists", c)
    }

    private fun showSettings() {
        val c = column()
        c.addView(label("Headlight brightness"))
        c.addView(detach(brightSeek))
        c.addView(detach(rampLabel))
        c.addView(detach(rampSeek))
        c.addView(label("Throttle pad: move to the right"))
        c.addView(detach(padSeek))
        c.addView(label("Steering pad: move up"))
        c.addView(detach(raiseSeek))
        c.addView(row(chip("Calibrate the car's sensors") {
            closePopup()
            showCalibrate()
        }))
        popup("Settings", c)
    }

    private fun showCalibrate() {
        val c = column()
        c.addView(note(
            "1. Put the car on a flat surface, keep it still, tap Level and zero. Roll and pitch should drop to about 0.\n" +
            "2. Turn the car to the left by hand. Yaw should go up. If it goes down, tap Flip yaw direction.\n" +
            "The result is saved in the car."
        ))
        c.addView(detach(calInfo))
        c.addView(row(
            chip("Level and zero") {
                requireCar {
                    link.sendQueued("C,1")
                    toast("Keep the car still for a second")
                }
            },
            chip("Zero yaw") { requireCar { link.sendQueued("Z") } }
        ))
        c.addView(row(
            chip("Flip yaw direction") { requireCar { link.sendQueued("N") } },
            chip("Clear saved level") { requireCar { link.sendQueued("C,0") } }
        ))
        popup("Calibrate", c)
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

    private fun render() {
        val t = latest
        val connected = linkState == CarLink.State.CONNECTED
        val fresh = connected && SystemClock.elapsedRealtime() - lastTelemetryAt < 1500

        statusText.text = when {
            fresh -> linkMessage
            connected -> "Waiting for the car..."
            linkState == CarLink.State.CONNECTING -> "Connecting to $linkMessage..."
            else -> linkMessage
        }
        statusText.setTextColor(if (fresh) GOOD else if (connected) WARN else MUTED)
        connectChip.text = if (connected) "Disconnect" else "Connect"

        if (fresh) {
            if (t.battV == 0f) {
                batteryM.value.text = "n/a"
                batteryM.value.setTextColor(MUTED)
            } else if (t.battV < 3f) {
                batteryM.value.text = "USB only"
                batteryM.value.setTextColor(MUTED)
            } else {
                batteryM.value.text = String.format(Locale.US, "%.2f V", t.battV)
                batteryM.value.setTextColor(if (t.battV < 6.8f) BAD else if (t.battV < 7.2f) WARN else GOOD)
            }
            speedM.value.text = "${t.speed}%"
            tiltM.value.text = "R${t.roll} P${t.pitch}"
            calInfo.text = "Roll ${t.roll}   Pitch ${t.pitch}   Yaw ${t.yaw}"
        } else {
            for (m in listOf(batteryM, speedM, tiltM)) {
                m.value.text = "-"
                m.value.setTextColor(MUTED)
            }
            calInfo.text = "Not connected"
        }
        sonarView.setData(t.distC, t.distL, t.distR, fresh)

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
        val stabOn = f and FLAG_STAB != 0
        val headingOn = f and FLAG_HEADING != 0
        styleChip(stabChip, stabOn)
        styleChip(headingChip, headingOn)
        val holding = f and FLAG_HOLD != 0
        styleChip(holdChip, holdArmed || holding, if (holding) WARN else ACCENT)
        styleChip(tiltChip, tiltOn)

        // popup buttons show the current state
        modesBtn.text = if (fresh) "Mode: " + MODE_NAMES.getOrElse(t.mode) { "?" } else "Mode"
        autoBtn.text = if (fresh) "Auto: " + AUTO_NAMES.getOrElse(t.auto) { "?" } else "Auto"
        val assists = (if (stabOn) 1 else 0) + (if (headingOn) 1 else 0)
        assistBtn.text = if (assists > 0) "Assists ($assists on)" else "Assists"
        styleChip(modesBtn, false)
        styleChip(autoBtn, fresh && t.auto != 0)
        styleChip(assistBtn, assists > 0)
        styleChip(settingsBtn, false)
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
