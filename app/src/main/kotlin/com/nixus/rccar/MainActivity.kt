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
import android.graphics.Typeface
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

// Colours follow the car's own jumper wires: green throttle, yellow steering, red brake,
// orange lights, blue assists, purple autonomy, on a neutral graphite background.
private val BG = 0xFF111315.toInt()
private val CARD = 0xFF1A1D20.toInt()
private val BORDER = 0xFF2C3034.toInt()
private val TEXT = 0xFFECEAE6.toInt()
private val MUTED = 0xFF8C9096.toInt()
private val DARK = 0xFF0B0D0E.toInt()
private val THROTTLE = 0xFF35C46A.toInt()
private val STEER = 0xFFF2C230.toInt()
private val BRAKE = 0xFFEF4B3F.toInt()
private val LIGHT = 0xFFFF8A2B.toInt()
private val ASSIST = 0xFF4C8DFF.toInt()
private val AUTO = 0xFFA77BFF.toInt()
private val NEUTRAL = 0xFFB8BDC4.toInt()

private const val FLAG_LINK_LOST = 1
private const val FLAG_LIMP = 2
private const val FLAG_CUTOFF = 4
private const val FLAG_FLIPPED = 8
private const val FLAG_BLOCKED = 16
private const val FLAG_CRASH = 32
private const val FLAG_HOLD = 64
private const val FLAG_STAB = 128
private const val FLAG_HEADING = 256
private const val FLAG_STUCK = 512
private const val FLAG_EMERGENCY = 1024
private const val FLAG_HARD_BRAKE = 2048
private const val FLAG_LIMITED = 4096
private const val FLAG_HILL = 8192
private const val FLAG_DRIFT = 16384
private const val FLAG_GAVE_UP = 32768

private const val FLAG2_TRACTION = 1
private const val FLAG2_ABS = 2
private const val FLAG2_STEER_AROUND = 4
private const val FLAG2_SONAR_ON = 8
private const val FLAG2_COAST_BRAKE = 16
private val TRACTION_DEFAULTS = intArrayOf(0, 30, 80)          // Eco, Normal, Sport
private val AUTO_MAX_DEFAULTS = intArrayOf(100, 50, 45, 45)    // index 1..3 = Roam, Follow, Corridor
private const val DEFAULT_ABS_MODES = 6                        // Normal and Sport

private const val REQ_BT = 1
private const val TILT_MAX_DEG = 40f
private const val TILT_DEADZONE_DEG = 3f

private const val STEER_W_DP = 330
private const val STEER_H_DP = 150
private const val THROTTLE_W_DP = 120
private const val BOTTOM_ROW_DP = 52
private const val DEFAULT_PAD_LEFT = 40
private const val DEFAULT_STEER_RAISE = 8
private const val DEFAULT_PAD_TRAVEL = 70
private const val DEFAULT_STEER_RETURN = 500
private const val DEFAULT_GAP = 100
private const val ALL_ASSISTS = 127
private val MODE_NAMES = arrayOf("Eco", "Normal", "Sport")
private val AUTO_NAMES = arrayOf("Manual", "Roam", "Follow", "Corridor")

class MainActivity : Activity(), CarLink.Listener, SensorEventListener {

    private val ui = Handler(Looper.getMainLooper())
    private val link = CarLink(this)

    @Volatile private var latest = Telemetry()
    @Volatile private var lastTelemetryAt = 0L
    private var linkState = CarLink.State.DISCONNECTED
    private var linkMessage = "Not connected"

    private val autopilot = Autopilot()
    private var autoOwnsDrive = false
    private var curThrottle = 0
    private var curSteer = 0
    private var padBrake = false
    private var btnBrake = false
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
    private lateinit var infoBtn: TextView
    private lateinit var alertText: TextView
    private lateinit var calInfo: TextView
    private lateinit var motionInfo: TextView
    private lateinit var diagText: TextView
    private lateinit var batteryM: Metric
    private lateinit var speedM: Metric
    private lateinit var tiltM: Metric
    private lateinit var modeChips: List<TextView>
    private lateinit var autoChips: List<TextView>
    private lateinit var assistChips: List<Pair<TextView, Int>>
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
    private lateinit var brakeChip: TextView
    private lateinit var rampLabel: TextView
    private lateinit var rampSeek: SeekBar
    private lateinit var brightSeek: SeekBar
    private lateinit var padSeek: SeekBar
    private lateinit var raiseSeek: SeekBar
    private lateinit var travelLabel: TextView
    private lateinit var travelSeek: SeekBar
    private lateinit var returnLabel: TextView
    private lateinit var returnSeek: SeekBar
    private lateinit var gapLabel: TextView
    private lateinit var gapSeek: SeekBar
    private lateinit var diffLabel: TextView
    private lateinit var diffSeek: SeekBar
    private var diffPct = 100
    private var padTravel = DEFAULT_PAD_TRAVEL
    private var steerReturn = DEFAULT_STEER_RETURN
    private var gapPct = DEFAULT_GAP
    private var assistMask = ALL_ASSISTS
    private val tractionByMode = IntArray(3)
    private var absModes = DEFAULT_ABS_MODES
    private var sonarEnabled = true
    private val autoMax = IntArray(4)
    private var tractionShownMode = 1
    private lateinit var tractionLabel: TextView
    private lateinit var tractionSeek: SeekBar
    private lateinit var absModeChips: List<Pair<TextView, Int>>
    private lateinit var sonarChip: TextView
    private lateinit var autoMaxLabels: List<TextView>
    private lateinit var autoMaxSeeks: List<SeekBar>
    private lateinit var sensors: SensorManager

    // layout pieces whose margins the position sliders move
    private lateinit var throttleLp: FrameLayout.LayoutParams
    private lateinit var steerLp: FrameLayout.LayoutParams
    private lateinit var centerLp: FrameLayout.LayoutParams
    private lateinit var centerBox: LinearLayout
    private var padLeftDp = DEFAULT_PAD_LEFT
    private var steerRaiseDp = DEFAULT_STEER_RAISE
    private var popupDialog: AlertDialog? = null

    private val lastStyle = HashMap<TextView, Int>()

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
        ui.post(autoTask)
        if (tiltOn) registerTilt()
    }

    override fun onPause() {
        ui.removeCallbacks(renderTask)
        ui.removeCallbacks(autoTask)
        autoOwnsDrive = false
        link.sendQueued("A,0")
        sensors.unregisterListener(this)
        curThrottle = 0
        padBrake = false
        btnBrake = false
        throttlePad.setExternal(0)
        link.setDrive(0, if (tiltOn) 0 else curSteer, false)
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

    private fun alpha(color: Int, a: Int): Int = (color and 0x00FFFFFF) or (a shl 24)

    /**
     * on = filled with the chip's colour; armed = outlined in it (ready but not active); otherwise a quiet
     * card with a faint outline of its colour, so the colour coding is visible even when everything is off.
     */
    private fun styleChip(tv: TextView, on: Boolean, armed: Boolean = false) {
        val key = (if (on) 1 else 0) or (if (armed) 2 else 0)
        if (lastStyle[tv] == key) return
        lastStyle[tv] = key
        val accent = (tv.tag as? Int) ?: ACCENT_DEFAULT
        val g = GradientDrawable()
        g.cornerRadius = dp(10).toFloat()
        when {
            on -> {
                g.setColor(accent)
                g.setStroke(dp(1), accent)
                tv.setTextColor(DARK)
            }
            armed -> {
                g.setColor(CARD)
                g.setStroke(dp(2), accent)
                tv.setTextColor(accent)
            }
            else -> {
                g.setColor(CARD)
                g.setStroke(dp(1), alpha(accent, 0x66))
                tv.setTextColor(TEXT)
            }
        }
        tv.background = g
    }

    private val ACCENT_DEFAULT = ASSIST

    private fun chip(label: String, accent: Int = NEUTRAL, size: Float = 13f, onClick: () -> Unit): TextView {
        val tv = TextView(this)
        tv.text = label
        tv.textSize = size
        tv.gravity = Gravity.CENTER
        tv.minHeight = dp(42)
        tv.setPadding(dp(6), dp(6), dp(6), dp(6))
        tv.tag = accent
        tv.setOnClickListener { onClick() }
        styleChip(tv, false)
        return tv
    }

    private fun metric(title: String, valueSize: Float, condensed: Boolean): Metric {
        val box = LinearLayout(this)
        box.orientation = LinearLayout.VERTICAL
        box.setPadding(dp(10), dp(4), dp(10), dp(4))
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
        v.textSize = valueSize
        if (condensed) v.typeface = Typeface.create("sans-serif-condensed", Typeface.BOLD)
        box.addView(t)
        box.addView(v)
        return Metric(box, v)
    }

    /** A view can only have one parent, so take it out of the old one (for example an earlier popup) first. */
    private fun <T : View> detach(v: T): T {
        (v.parent as? ViewGroup)?.removeView(v)
        return v
    }

    private fun row(vararg views: View): LinearLayout = rowW(floatArrayOf(), *views)

    private fun rowW(weights: FloatArray, vararg views: View): LinearLayout {
        val r = LinearLayout(this)
        r.orientation = LinearLayout.HORIZONTAL
        for ((i, v) in views.withIndex()) {
            detach(v)
            val w = if (i < weights.size) weights[i] else 1f
            val lp = LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, w)
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

    private fun seekColors(s: SeekBar, color: Int = NEUTRAL) {
        s.progressTintList = ColorStateList.valueOf(color)
        s.thumbTintList = ColorStateList.valueOf(color)
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

    /** One slider that remembers its value and sends it to the car when it moves. */
    private fun slider(max: Int, start: Int, color: Int, onChange: (Int, Boolean) -> Unit, onDone: (Int) -> Unit): SeekBar {
        val s = SeekBar(this)
        s.max = max
        s.progress = start
        seekColors(s, color)
        s.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar, p: Int, fromUser: Boolean) = onChange(p, fromUser)
            override fun onStartTrackingTouch(sb: SeekBar) {}
            override fun onStopTrackingTouch(sb: SeekBar) = onDone(sb.progress)
        })
        return s
    }

    // ------------------------------------------------------------------ UI

    private fun buildUi() {
        padLeftDp = prefs().getInt("padLeft", DEFAULT_PAD_LEFT).coerceIn(0, 120)
        steerRaiseDp = prefs().getInt("steerRaise", DEFAULT_STEER_RAISE).coerceIn(0, 100)
        diffPct = prefs().getInt("diffPct", 100).coerceIn(50, 150)
        padTravel = prefs().getInt("padTravel", DEFAULT_PAD_TRAVEL).coerceIn(40, 110)
        steerReturn = prefs().getInt("steerReturn", DEFAULT_STEER_RETURN).coerceIn(0, 1500)
        gapPct = prefs().getInt("gapPct", DEFAULT_GAP).coerceIn(50, 200)
        assistMask = prefs().getInt("assistMask", ALL_ASSISTS) and ALL_ASSISTS
        for (i in 0..2) tractionByMode[i] = prefs().getInt("traction$i", TRACTION_DEFAULTS[i]).coerceIn(0, 100)
        absModes = prefs().getInt("absModes", DEFAULT_ABS_MODES) and 7
        sonarEnabled = prefs().getBoolean("sonarOn", true)
        for (i in 1..3) autoMax[i] = prefs().getInt("autoMax$i", AUTO_MAX_DEFAULTS[i]).coerceIn(15, 100)

        // ---- pads
        throttlePad = AxisPad(this, true)
        throttlePad.accent = THROTTLE
        throttlePad.brakeColor = BRAKE
        throttlePad.reverseColor = LIGHT
        throttlePad.title = "Throttle"
        throttlePad.hintMax = "forward"
        throttlePad.hintMin = "reverse"
        throttlePad.hintSide = "swipe left to brake"
        throttlePad.travelDp = padTravel.toFloat()
        throttlePad.onValue = { v ->
            curThrottle = v
            pushDrive()
        }
        throttlePad.onGrab = { down ->
            // touching the throttle again cancels a held speed
            if (down && (latest.flags and FLAG_HOLD) != 0) link.sendQueued("K,0")
        }
        throttlePad.onRelease = { last ->
            if (holdArmed && last > 0 && latest.auto == 0 && !padBrake) {
                // lock the speed the car is doing right now, then let go of the pad
                link.sendQueued("D,$last,$curSteer,0")
                link.sendQueued("K,1")
            }
        }
        throttlePad.onBrake = { b ->
            padBrake = b
            if (b) {
                holdArmed = false         // braking ends cruise
                curThrottle = 0
            }
            pushDrive()
        }

        steerPad = AxisPad(this, false)
        steerPad.accent = STEER
        steerPad.title = "Steering"
        steerPad.hintMin = "left"
        steerPad.hintMax = "right"
        steerPad.travelDp = padTravel.toFloat()
        steerPad.onValue = { v ->
            if (!tiltOn) {
                curSteer = v
                pushDrive()
            }
        }

        // ---- status row: connect, message, information button
        statusText = TextView(this)
        statusText.setTextColor(TEXT)
        statusText.textSize = 13f
        statusText.gravity = Gravity.CENTER_VERTICAL
        connectChip = chip("Connect", NEUTRAL, 12f) { onConnectClicked() }
        infoBtn = TextView(this)
        infoBtn.text = "i"
        infoBtn.textSize = 18f
        infoBtn.typeface = Typeface.create(Typeface.SERIF, Typeface.BOLD_ITALIC)
        infoBtn.gravity = Gravity.CENTER
        infoBtn.setTextColor(TEXT)
        val ig = GradientDrawable()
        ig.shape = GradientDrawable.OVAL
        ig.setColor(CARD)
        ig.setStroke(dp(1), MUTED)
        infoBtn.background = ig
        infoBtn.setOnClickListener { showManual() }
        val statusRow = LinearLayout(this)
        statusRow.orientation = LinearLayout.HORIZONTAL
        statusRow.gravity = Gravity.CENTER_VERTICAL
        val cl = LinearLayout.LayoutParams(dp(100), ViewGroup.LayoutParams.WRAP_CONTENT)
        cl.setMargins(0, 0, dp(8), 0)
        statusRow.addView(connectChip, cl)
        statusRow.addView(statusText, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        statusRow.addView(infoBtn, LinearLayout.LayoutParams(dp(36), dp(36)))

        speedM = metric("Speed %", 28f, true)
        batteryM = metric("Battery", 15f, false)
        tiltM = metric("Tilt", 15f, false)
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

        motionInfo = TextView(this)
        motionInfo.setTextColor(TEXT)
        motionInfo.textSize = 13f
        motionInfo.setPadding(dp(4), dp(6), dp(4), dp(6))

        diagText = TextView(this)
        diagText.setTextColor(TEXT)
        diagText.textSize = 12.5f
        diagText.typeface = Typeface.MONOSPACE
        diagText.setPadding(dp(4), dp(8), dp(4), dp(8))

        // ---- mode and auto (live inside popups)
        modeChips = MODE_NAMES.mapIndexed { i, n ->
            chip(n, THROTTLE) {
                link.sendQueued("M,$i")
                link.sendQueued("Q,${tractionByMode[i]}")          // a mode change resets the car's traction strength: send ours
                val defaults = intArrayOf(600, 300, 150)
                rampSeek.progress = defaults[i] / 50
                closePopup()
            }
        }
        autoChips = AUTO_NAMES.mapIndexed { i, n ->
            chip(n, AUTO) {
                if (i != 0 && !sonarEnabled) {
                    toast("Sonars are off. Turn them on in Assists first")
                    return@chip
                }
                link.sendQueued("A,$i")
                closePopup()
            }
        }

        // ---- always-visible toggles
        lightsChip = chip("Lights", LIGHT, 12f) {
            lightsOn = !lightsOn
            link.sendQueued("L,${if (lightsOn) 1 else 0}")
            styleChip(lightsChip, lightsOn)
        }
        hazardChip = chip("Hazard", LIGHT, 12f) {
            hazardOn = !hazardOn
            link.sendQueued("H,${if (hazardOn) 1 else 0}")
            styleChip(hazardChip, hazardOn)
        }
        holdChip = chip("Cruise", THROTTLE, 12f) {
            holdArmed = !holdArmed
            if (!holdArmed && (latest.flags and FLAG_HOLD) != 0) link.sendQueued("K,0")
        }
        tiltChip = chip("Tilt steer", STEER, 12f) { toggleTilt() }

        // ---- horn and brake sit under the steering pad
        val hornChip = chip("Horn (hold)", LIGHT) { }
        hornChip.minHeight = dp(46)
        hornChip.setOnTouchListener { _, e ->
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    link.sendQueued("V,1")
                    styleChip(hornChip, true)
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    link.sendQueued("V,0")
                    styleChip(hornChip, false)
                }
            }
            true
        }
        // the brake button holds the full short brake for as long as it is pressed
        brakeChip = chip("BRAKE (hold)", BRAKE) { }
        brakeChip.minHeight = dp(46)
        brakeChip.setOnTouchListener { _, e ->
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    btnBrake = true
                    holdArmed = false
                    styleChip(brakeChip, true)
                    pushDrive()
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    btnBrake = false
                    styleChip(brakeChip, false)
                    pushDrive()
                }
            }
            true
        }

        // ---- assist toggles that live in the car's own state
        stabChip = chip("Stability assist", ASSIST) {
            val on = (latest.flags and FLAG_STAB) != 0
            link.sendQueued("S,${if (on) 0 else 1}")
        }
        headingChip = chip("Heading hold", ASSIST) {
            val on = (latest.flags and FLAG_HEADING) != 0
            link.sendQueued("G,${if (on) 0 else 1}")
        }
        tiltRevChip = chip("Reverse tilt", STEER) {
            tiltReverse = !tiltReverse
            styleChip(tiltRevChip, tiltReverse)
        }
        // ---- assist switches that the app remembers and sends as one mask
        val defs = listOf(
            1 to "Collision protection", 2 to "Emergency steering", 4 to "Wall keeping", 8 to "Hill hold",
            16 to "ABS guard", 32 to "Stuck cut-off", 64 to "Launch assist"
        )
        assistChips = defs.map { (bit, name) ->
            val c = chip(name, ASSIST) { toggleAssist(bit) }
            c to bit
        }

        // ---- ABS per drive mode, sonar switch, traction strength
        absModeChips = MODE_NAMES.mapIndexed { i, n -> chip(n, ASSIST) { toggleAbsMode(i) } to i }
        sonarChip = chip("Sonars on", ASSIST) { toggleSonar() }
        tractionLabel = label(tractionText(tractionShownMode))
        tractionSeek = slider(100, tractionByMode[tractionShownMode], THROTTLE, { p, user ->
            val m = tractionShownMode.coerceIn(0, 2)
            if (user) tractionByMode[m] = p
            tractionLabel.text = tractionText(m)
            if (user) link.sendLatest("Q", "Q,$p")
        }, { p ->
            link.sendLatest("Q", "Q,$p")
            prefs().edit().putInt("traction${tractionShownMode.coerceIn(0, 2)}", p).apply()
        })
        autoMaxLabels = (1..3).map { label(autoMaxText(it)) }
        autoMaxSeeks = (1..3).map { i ->
            slider(85, autoMax[i] - 15, AUTO, { p, user ->
                autoMax[i] = 15 + p
                autoMaxLabels[i - 1].text = autoMaxText(i)
                if (user) link.sendLatest("E$i", "E,$i,${autoMax[i]}")
            }, { _ ->
                link.sendLatest("E$i", "E,$i,${autoMax[i]}")
                prefs().edit().putInt("autoMax$i", autoMax[i]).apply()
            })
        }

        // ---- buttons that open popups (right column, above the steering pad)
        modesBtn = chip("Mode", THROTTLE) { showModes() }
        autoBtn = chip("Auto", AUTO) { showAuto() }
        assistBtn = chip("Assists", ASSIST) { showAssists() }
        settingsBtn = chip("Settings", NEUTRAL) { showSettings() }

        // ---- sliders (live inside popups)
        brightSeek = slider(100, 100, LIGHT, { p, user -> if (user) link.sendLatest("B", "B,$p") }, { p ->
            link.sendLatest("B", "B,$p")
        })
        rampLabel = label("Acceleration ramp: 300 ms")
        rampSeek = slider(30, 6, THROTTLE, { p, user ->
            rampLabel.text = "Acceleration ramp: ${p * 50} ms" + if (p == 0) " (launch mode)" else ""
            if (user) link.sendLatest("R", "R,${p * 50}")
        }, { p -> link.sendLatest("R", "R,${p * 50}") })

        diffLabel = label("Differential strength: $diffPct%")
        diffSeek = slider(100, diffPct - 50, THROTTLE, { p, user ->
            diffPct = 50 + p
            diffLabel.text = "Differential strength: $diffPct%"
            if (user) link.sendLatest("F", "F,$diffPct")
        }, { _ ->
            link.sendLatest("F", "F,$diffPct")
            prefs().edit().putInt("diffPct", diffPct).apply()
        })

        travelLabel = label("Pad travel: $padTravel dp for full throttle or steering")
        travelSeek = slider(70, padTravel - 40, NEUTRAL, { p, user ->
            padTravel = 40 + p
            travelLabel.text = "Pad travel: $padTravel dp for full throttle or steering"
            if (user) applyTravel()
        }, { _ ->
            applyTravel()
            prefs().edit().putInt("padTravel", padTravel).apply()
        })

        returnLabel = label(returnText())
        returnSeek = slider(30, steerReturn / 50, STEER, { p, user ->
            steerReturn = p * 50
            returnLabel.text = returnText()
            if (user) link.sendLatest("U", "U,$steerReturn")
        }, { _ ->
            link.sendLatest("U", "U,$steerReturn")
            prefs().edit().putInt("steerReturn", steerReturn).apply()
        })

        gapLabel = label("Following gap: $gapPct%")
        gapSeek = slider(150, gapPct - 50, ASSIST, { p, user ->
            gapPct = 50 + p
            gapLabel.text = "Following gap: $gapPct%"
            if (user) link.sendLatest("J", "J,$gapPct")
        }, { _ ->
            link.sendLatest("J", "J,$gapPct")
            prefs().edit().putInt("gapPct", gapPct).apply()
        })

        padSeek = slider(120, padLeftDp, NEUTRAL, { p, user ->
            if (user) {
                padLeftDp = p
                applyPadPositions()
            }
        }, { _ -> prefs().edit().putInt("padLeft", padLeftDp).apply() })
        raiseSeek = slider(100, steerRaiseDp, NEUTRAL, { p, user ->
            if (user) {
                steerRaiseDp = p
                applyPadPositions()
            }
        }, { _ -> prefs().edit().putInt("steerRaise", steerRaiseDp).apply() })

        // ---- center column: status, readings, sonar picture, alerts, toggles
        centerBox = LinearLayout(this)
        centerBox.orientation = LinearLayout.VERTICAL
        centerBox.addView(statusRow)
        centerBox.addView(rowW(floatArrayOf(1.25f, 1f, 1f), speedM.root, batteryM.root, tiltM.root))
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

    private fun returnText(): String =
        if (steerReturn == 0) "Steering return: instant" else "Steering return: $steerReturn ms to center"

    private fun applyTravel() {
        throttlePad.travelDp = padTravel.toFloat()
        steerPad.travelDp = padTravel.toFloat()
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

    private fun tractionText(mode: Int): String {
        val m = mode.coerceIn(0, 2)
        val v = tractionByMode[m]
        return "Traction control, ${MODE_NAMES[m]}: " + if (v == 0) "off" else "$v%"
    }

    private fun autoMaxText(i: Int): String = "${AUTO_NAMES[i]} top speed: ${autoMax[i]}%"

    private fun toggleAbsMode(i: Int) {
        absModes = absModes xor (1 shl i)
        prefs().edit().putInt("absModes", absModes).apply()
        link.sendLatest("Y", "Y,$absModes")
    }

    private fun toggleSonar() {
        sonarEnabled = !sonarEnabled
        prefs().edit().putBoolean("sonarOn", sonarEnabled).apply()
        link.sendLatest("O", "O,${if (sonarEnabled) 1 else 0}")
        if (!sonarEnabled) toast("Sonars off: no collision protection and no Auto modes")
    }

    private fun toggleAssist(bit: Int) {
        assistMask = assistMask xor bit
        prefs().edit().putInt("assistMask", assistMask).apply()
        link.sendLatest("T", "T,$assistMask")
    }

    // ------------------------------------------------------------------ popups

    private fun showModes() {
        val c = column()
        c.addView(label("Drive mode"))
        c.addView(row(modeChips[0], modeChips[1], modeChips[2]))
        c.addView(note("Top speed: Eco 55%, Normal 80%, Sport 100%. Picking a mode also sets a matching acceleration ramp and traction control strength (Eco off, Normal 30%, Sport 80%; change it in Assists)."))
        popup("Drive mode", c)
    }

    private fun showAuto() {
        val c = column()
        c.addView(row(autoChips[0], autoChips[1]))
        c.addView(row(autoChips[2], autoChips[3]))
        c.addView(label("Top speed of each mode"))
        for (i in 1..3) {
            c.addView(detach(autoMaxLabels[i - 1]))
            c.addView(detach(autoMaxSeeks[i - 1]))
        }
        c.addView(note("Auto modes need the sonars (Assists > Sonars). Roam: drives around and steers away from walls. Follow: follows something in front of it. Corridor: keeps to the middle between two walls. The car avoids reversing in all of them. Pick Manual or press the brake to take over. They stop if the link drops."))
        popup("Autonomous", c)
    }

    private fun showAssists() {
        val c = column()
        c.addView(label("Collision and motion"))
        c.addView(row(assistChips[0].first, assistChips[1].first))
        c.addView(row(assistChips[2].first, assistChips[3].first))
        c.addView(row(assistChips[4].first, assistChips[5].first))
        c.addView(row(assistChips[6].first))
        c.addView(label("Sonars"))
        c.addView(row(sonarChip))
        c.addView(note("Off: the front sonars are not read at all. There is no collision protection, wall keeping, Cruise gap, Auto mode or motion calibration, and the car will hit what is in front of it."))
        c.addView(label("Traction and ABS"))
        c.addView(detach(tractionLabel))
        c.addView(detach(tractionSeek))
        c.addView(note("Traction control (launch only) cuts the power for a moment when the wheels spin and the car is not picking up speed, then gives it back as they grip. Each mode keeps its own strength. Needs Motion and braking calibration."))
        c.addView(label("ABS in these drive modes (needs ABS guard on)"))
        c.addView(row(absModeChips[0].first, absModeChips[1].first, absModeChips[2].first))
        c.addView(note("At speed ABS pulses the brake so the wheels keep gripping, and once the car has slowed down it applies the full brake. It also pulses if the car starts to spin. Sport and Normal are on by default."))
        c.addView(label("Handling"))
        c.addView(row(stabChip, headingChip))
        c.addView(row(tiltRevChip))
        c.addView(detach(gapLabel))
        c.addView(detach(gapSeek))
        c.addView(note("Collision protection limits speed so the car can always stop, lets you steer around things, and brakes hard if a crash is unavoidable. Before that, if there is free space on one side, it slows down and steers around the obstacle early, then hands back to you (push the steering pad hard the other way to take over). Emergency steering is the last resort: the car steers to the free side for about a second. Wall keeping nudges you away from walls at the sides. ABS guard is the master switch for ABS. Stuck cut-off drops the power when the car is pushing but not moving. Following gap is the distance Cruise keeps behind things; it grows with speed."))
        popup("Assists", c)
    }

    private fun showSettings() {
        val c = column()
        c.addView(label("Headlight brightness"))
        c.addView(detach(brightSeek))
        c.addView(detach(rampLabel))
        c.addView(detach(rampSeek))
        c.addView(detach(diffLabel))
        c.addView(detach(diffSeek))
        c.addView(note("100% is the exact turning geometry of your car, and the car turns tighter at low speed. Raise it if the car turns wider than the steering asks for, lower it if the wheels scrub or the car spins out."))
        c.addView(detach(returnLabel))
        c.addView(detach(returnSeek))
        c.addView(detach(travelLabel))
        c.addView(detach(travelSeek))
        c.addView(label("Throttle pad: move to the right"))
        c.addView(detach(padSeek))
        c.addView(label("Steering pad: move up"))
        c.addView(detach(raiseSeek))
        c.addView(row(
            chip("Calibrate") {
                closePopup()
                showCalibrate()
            },
            chip("Diagnostics") {
                closePopup()
                showDiagnostics()
            }
        ))
        popup("Settings", c)
    }

    private fun showCalibrate() {
        val c = column()
        c.addView(label("1. Level and yaw"))
        c.addView(note(
            "Put the car on a flat surface, keep it still, tap Level and zero. Roll and pitch should drop to about 0. " +
            "Then turn the car to the left by hand: yaw should go up. If it goes down, tap Flip yaw direction."
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
        c.addView(label("2. Motion and braking"))
        c.addView(note(
            "The car drives about 2 m at full power toward a wall, then brakes hard. It learns which way is forward for its motion sensor, " +
            "its real speed and its real braking rate. Do Level and zero first. Needs a flat wall 280 to 400 cm straight ahead and a clear, " +
            "grippy floor. Hold BRAKE to stop it. Every number it measured shows in Diagnostics, also when it fails."
        ))
        c.addView(detach(motionInfo))
        c.addView(row(chip("Start motion and braking calibration", AUTO) { confirmMotionCal() }))
        popup("Calibrate", c)
    }

    private fun confirmMotionCal() {
        requireCar {
            if (!sonarEnabled) {
                toast("Turn the sonars on first (Assists)")
                return@requireCar
            }
            AlertDialog.Builder(this, android.R.style.Theme_Material_Dialog_Alert)
                .setTitle("The car will drive")
                .setMessage("It drives about 2 m forward at full power, then brakes hard. Put it on the floor with a flat wall 280 to 400 cm straight ahead (about 3 m) and nothing in the way, and stand clear. Hold BRAKE to stop it.")
                .setPositiveButton("Start") { _, _ -> link.sendQueued("C,2") }
                .setNegativeButton("Cancel") { _, _ -> }
                .show()
        }
    }

    private fun showDiagnostics() {
        val c = column()
        c.addView(detach(diagText))
        c.addView(note("Turn rate: measured by the gyro, then what the steering should give at this speed. A big gap with the wheels straight means the car is drifting or sliding. Own speed is the larger of the motor model and the motion sensor. Last calibration run lists the raw numbers of the latest motion and braking calibration, so a failure shows which check it missed."))
        popup("Diagnostics", c)
    }

    private fun showManual() {
        val c = column()
        fun section(title: String, body: String) {
            val h = TextView(this)
            h.text = title
            h.setTextColor(TEXT)
            h.textSize = 15f
            h.typeface = Typeface.create("sans-serif-medium", Typeface.NORMAL)
            h.setPadding(dp(4), dp(16), dp(4), dp(2))
            c.addView(h)
            val b = TextView(this)
            b.text = body
            b.setTextColor(MUTED)
            b.textSize = 13f
            b.setLineSpacing(0f, 1.15f)
            b.setPadding(dp(4), dp(2), dp(4), dp(2))
            c.addView(b)
        }
        section("Quick start",
            "1. Switch the car on and pair its Bluetooth module in Android settings (PIN 1234).\n" +
            "2. Tap Connect and pick the module. The app reconnects by itself next time.\n" +
            "3. First time only: put the car on a flat floor, open Settings > Calibrate, tap Level and zero, then run Motion and braking calibration (it needs a wall about 3 m ahead).\n" +
            "4. Put your left thumb on the throttle pad and push up.")
        section("Throttle pad (left)",
            "Put your thumb anywhere on the pad. Where it lands is zero, marked by a ring. Push up to go forward, down to reverse. " +
            "A small dead zone around the ring ignores accidental slips, so a light tap never moves the car. " +
            "Full throttle is at the distance set in Settings > Pad travel (smaller = more sensitive). Lift your thumb and the throttle returns to zero.")
        section("Steering pad (right)",
            "Same idea: where your thumb lands is straight ahead. Slide left or right. " +
            "When you let go, the steering eases back to center over the time set in Settings > Steering return, which spares the servo. " +
            "Tilt steer uses the phone like a steering wheel instead; Reverse tilt in Assists swaps the direction.")
        section("Braking",
            "While your thumb is on the throttle, slide it left past the red brake line. The car shorts both motors for the hardest stop it can do and holds it while you stay left. " +
            "Slide back or lift to release. After a brake the throttle stays at zero until you lift or come back to the ring, so you can't lurch forward.\n" +
            "The BRAKE button under the steering pad does the same while pressed. Braking ends Cruise and any Auto mode.\n" +
            "ABS (Assists) works per drive mode. Above a walking pace it pulses the brake so the wheels keep gripping, and once the car has slowed down it applies the full brake. It also pulses if the gyro sees the car start to spin. ABS guard is the master switch; the Eco, Normal and Sport buttons choose where it is active.")
        section("Traction control",
            "On launch, if the wheels spin and the car is not picking up speed, the car cuts the power for a moment and gives it back as the wheels grip. " +
            "Each drive mode has its own strength (Eco off, Normal 30%, Sport 80%); change it in Assists. It reacts within a fraction of a second and needs Motion and braking calibration.")
        section("Mode, Auto, Assists, Settings",
            "Mode: Eco, Normal or Sport. They set the top speed (55, 80, 100%) and how quickly the car accelerates.\n" +
            "Auto: Roam, Follow or Corridor, each with its own top speed slider. The car avoids reversing: it tries a tight forward turn first, then a short, very slow reverse with beeps, then stops and tells you it needs help.\n" +
            "Assists: switches for the collision system and handling aids.\n" +
            "Settings: headlights, acceleration, differential, steering return, pad feel and position, Calibrate and Diagnostics.")
        section("Cruise",
            "Tap Cruise to get it ready, push the throttle to the speed you want, then let go: the car keeps that speed. " +
            "If something is ahead it slows down to keep a gap that grows with speed (Assists > Following gap) and speeds back up once the way is clear. " +
            "Touch the throttle or brake to cancel. Cruise works forward only.")
        section("Collision protection",
            "The car watches its three front sonars and its own speed, and it does this when you let go and it is rolling, too.\n" +
            "1. It limits your speed smoothly so it can always stop in time.\n" +
            "2. If there is free space on one side and you steer that way, it lets you go around.\n" +
            "3. With free space on one side it slows down and steers around the obstacle early, then hands back (Steering around it). You can override by pushing the steering pad hard the other way.\n" +
            "4. If a crash is unavoidable it brakes as hard as it can.\n" +
            "5. If braking will not be enough it steers to the free side for about a second, then hands back.\n" +
            "If the car is rolling or you give it a push toward something with no throttle, it brakes too.\n" +
            "There are no sensors at the back, so reversing is never protected. The sonars are narrow: thin poles and soft things can be missed.\n" +
            "Sonars off (Assists > Sonars): the sonars are not read at all. Collision protection, wall keeping, Cruise gap and the Auto modes stop working, and the car will hit what is in front of it.")
        section("Hill hold, launch assist, stuck",
            "Hill hold: stopped on a slope steeper than 5 degrees, the car keeps the brakes on.\n" +
            "Launch assist: adds a little throttle when you start uphill.\n" +
            "Stuck cut-off: if you give throttle and the car does not move for about a second, power is cut and Stuck shows. Let go of the throttle and push again.\n" +
            "Slip correction: if the car turns differently from what the steering asks for, the car steers against it and shows Slipping.")
        section("Calibrate",
            "Level and zero: car flat and still. Flip yaw direction if yaw goes down when you turn the car left.\n" +
            "Motion and braking calibration: with a flat wall 280 to 400 cm straight ahead (about 3 m) the car drives about 2 m at full power, then brakes hard. It learns which way forward is for the motion sensor, its real speed and its real braking rate, and saves them. Do it once, again if the board is moved, and after changing tyres or the floor. If it fails, the message says which check missed, and Diagnostics lists every number it measured.\n" +
            "Diagnostics shows live numbers: own speed, closing speed, turn rate against what steering should give, slope, the speed limit, traction and ABS activity, and the last calibration run.")
        section("What the alerts mean",
            "Link lost: the phone link dropped and the car stopped.\n" +
            "Battery cutoff or Limp mode: the pack is low.\n" +
            "Flipped or Crash: the motion sensor saw it; the motors stop.\n" +
            "Stuck: power was cut; release the throttle and push again.\n" +
            "Steering for you: emergency steering is active.\n" +
            "Steering around it: the car is steering around an obstacle on its own.\n" +
            "Braking: the collision system or you are braking hard. Rolling toward something means it is braking because the car is coasting or was pushed.\n" +
            "Traction control, ABS: the aid is working right now.\n" +
            "Sonars off: collision protection is disabled.\n" +
            "Slowing down: speed is limited because something is ahead.\n" +
            "Hill hold, Slipping, Cruise: those aids are working.\n" +
            "Auto mode needs help: it found no safe way forward and stopped.")
        section("If something is off",
            "Won't connect: the car must be on and paired first.\n" +
            "Pads feel twitchy or slow: change Pad travel in Settings.\n" +
            "Car turns wider or sharper than expected: change Differential strength.\n" +
            "The car stops too early or too late near walls: it needs real-floor tuning of the braking numbers in the sketch (see the README).\n" +
            "Nothing works after a crash of the app: reopen it; the last error is shown on screen.")
        popup("Manual", c)
    }

    private fun toast(msg: String) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
    }

    private fun pushDrive() {
        val brake = padBrake || btnBrake
        if (latest.auto != 0 && !brake) return      // the autopilot owns the drive line while an Auto mode runs
        link.setDrive(curThrottle, curSteer, brake)
    }

    // Roam / Follow / Corridor run here on the phone (they no longer fit in the Nano): sonar in, throttle and steering out.
    private val autoTask = object : Runnable {
        override fun run() {
            val t = latest
            val now = SystemClock.elapsedRealtime()
            if (t.auto != 0 && link.isConnected) {
                autoOwnsDrive = true
                if (now - lastTelemetryAt < 500L) {
                    autopilot.step(t, t.auto, autoMax[t.auto], now)
                    link.setDrive(autopilot.throttle, autopilot.steer, false)
                } else {
                    link.setDrive(0, 0, false)
                }
                if (autopilot.gaveUp) {
                    autopilot.clearGaveUp()
                    link.sendQueued("A,0,1")
                }
            } else if (autoOwnsDrive) {
                autoOwnsDrive = false
                pushDrive()
            }
            ui.postDelayed(this, 50)
        }
    }

    // ------------------------------------------------------------------ rendering (10 times a second)

    private val renderTask = object : Runnable {
        override fun run() {
            render()
            ui.postDelayed(this, 100)
        }
    }

    private fun calWhyText(t: Telemetry): String = when (t.calWhy) {
        1 -> "The motion sensor felt almost no push (${t.runPush} cm/s2). Check the board is fixed firmly to the chassis."
        2 -> "The car did not cover 2 m in time. Slipping wheels or a weak battery? Try a grippier floor or a charged pack."
        3 -> String.format(Locale.US, "Speed scale came out odd (%.2f cm/s per %%, expected 0.3 to 2.0). The wheels may have slipped.", t.runK)
        4 -> "Braking rate came out odd (${t.runDecel} cm/s2, expected 100 to 3000)."
        5 -> "Safety stop: the wall was closer than 60 cm before the run ended. Start farther back."
        6 -> "Stopped by throttle, brake, lost link or a flip."
        7 -> "The stopping distance was too short to measure and the motion sensor did not see the braking."
        else -> "The result looked odd."
    }

    private fun motionStatus(t: Telemetry, fresh: Boolean): String {
        if (!fresh) return "Not connected"
        return when (t.calState) {
            1 -> "Running: the car drives about 2 m, then brakes hard..."
            2 -> String.format(Locale.US, "Done. Speed scale %.2f cm/s per %%, braking rate %d cm/s2. Saved in the car.", t.kSpeed, t.brakeDecel)
            3 -> "No wall found 280 to 400 cm straight ahead (front sonar reads ${cm(t.distC)}). Move the car and try again."
            4 -> if (t.calWhy != 0) "Stopped: " + calWhyText(t) else "Stopped before it finished. Try again."
            5 -> "Failed: " + calWhyText(t) + " Details are in Diagnostics."
            else -> if (t.fwdKnown) "Learned and saved in the car." else "Not learned yet. The car works without it, but traction control, stuck detection and the speed estimate are limited."
        }
    }

    private fun flagNames(f: Int): String {
        val n = ArrayList<String>()
        if (f and FLAG_STUCK != 0) n.add("stuck")
        if (f and FLAG_EMERGENCY != 0) n.add("emergency steer")
        if (f and FLAG_HARD_BRAKE != 0) n.add("hard brake")
        if (f and FLAG_LIMITED != 0) n.add("speed limited")
        if (f and FLAG_HILL != 0) n.add("hill hold")
        if (f and FLAG_DRIFT != 0) n.add("slip")
        if (f and FLAG_HOLD != 0) n.add("cruise")
        if (f and FLAG_BLOCKED != 0) n.add("blocked")
        if (f and FLAG_GAVE_UP != 0) n.add("auto gave up")
        return if (n.isEmpty()) "none" else n.joinToString(", ")
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
        statusText.setTextColor(if (fresh) THROTTLE else if (connected) LIGHT else MUTED)
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
                batteryM.value.setTextColor(if (t.battV < 6.8f) BRAKE else if (t.battV < 7.2f) LIGHT else THROTTLE)
            }
            speedM.value.text = "${t.speed}"
            speedM.value.setTextColor(if (t.speed < 0) LIGHT else TEXT)
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
        val sonarsOn = if (fresh) t.flags2 and FLAG2_SONAR_ON != 0 else sonarEnabled
        sonarView.setOff(fresh && !sonarsOn)

        // alerts, most urgent first
        val f = t.flags
        val parts = ArrayList<String>()
        var danger = false
        var warn = false
        if (fresh) {
            fun add(text: String, level: Int) {
                parts.add(text)
                if (level == 2) danger = true
                if (level == 1) warn = true
            }
            if (f and FLAG_LINK_LOST != 0) add("Link lost, car stopped", 2)
            if (f and FLAG_CUTOFF != 0) add("Battery cutoff", 2)
            if (f and FLAG_FLIPPED != 0) add("Flipped", 2)
            if (f and FLAG_CRASH != 0) add("Crash detected", 2)
            if (f and FLAG_STUCK != 0) add("Stuck, power cut. Let go of the throttle and push again", 2)
            if (f and FLAG_GAVE_UP != 0) add("Auto mode needs help: no safe way forward", 2)
            if (f and FLAG_EMERGENCY != 0) add("Steering for you (push the pad hard the other way to take over)", 1)
            if (f and FLAG_HARD_BRAKE != 0) add("Braking", 1)
            if (t.flags2 and FLAG2_STEER_AROUND != 0) add("Steering around it", 1)
            if (t.flags2 and FLAG2_COAST_BRAKE != 0 && f and FLAG_HARD_BRAKE == 0) add("Braking: rolling toward something", 1)
            if (f and FLAG_LIMITED != 0) add("Slowing down: obstacle ahead (limit ${t.cap}%)", 1)
            if (t.flags2 and FLAG2_SONAR_ON == 0) add("Sonars off: no collision protection", 1)
            if (f and FLAG_LIMP != 0) add("Limp mode: battery low", 1)
            if (f and FLAG_BLOCKED != 0 && f and FLAG_LIMITED == 0) add("Obstacle ahead", 1)
            if (t.flags2 and FLAG2_TRACTION != 0) add("Traction control", 0)
            if (t.flags2 and FLAG2_ABS != 0) add("ABS", 0)
            if (f and FLAG_DRIFT != 0) add("Slipping, correcting", 0)
            if (f and FLAG_HILL != 0) add("Hill hold", 0)
            if (f and FLAG_HOLD != 0) add("Cruise on (touch throttle to release)", 0)
        }
        alertText.text = if (!fresh) "" else if (parts.isEmpty()) "All clear" else parts.take(2).joinToString("  |  ")
        alertText.setTextColor(if (!fresh) MUTED else if (danger) BRAKE else if (warn) LIGHT else if (parts.isEmpty()) THROTTLE else ASSIST)
        sonarView.setEdge(
            if (!fresh) 0
            else if (f and (FLAG_EMERGENCY or FLAG_HARD_BRAKE or FLAG_STUCK) != 0 || t.flags2 and FLAG2_COAST_BRAKE != 0) BRAKE
            else if (f and FLAG_LIMITED != 0) LIGHT
            else 0
        )

        // chip highlights follow what the car reports
        for (i in modeChips.indices) styleChip(modeChips[i], fresh && i == t.mode)
        for (i in autoChips.indices) styleChip(autoChips[i], fresh && i == t.auto)
        val stabOn = f and FLAG_STAB != 0
        val headingOn = f and FLAG_HEADING != 0
        styleChip(stabChip, stabOn)
        styleChip(headingChip, headingOn)
        val holding = f and FLAG_HOLD != 0
        holdChip.text = if (holding) "Cruise on" else if (holdArmed) "Cruise ready" else "Cruise"
        styleChip(holdChip, holding, holdArmed && !holding)
        styleChip(tiltChip, tiltOn)
        styleChip(tiltRevChip, tiltReverse)
        styleChip(lightsChip, lightsOn)
        styleChip(hazardChip, hazardOn)
        for ((c, bit) in assistChips) styleChip(c, assistMask and bit != 0)
        for ((c, i) in absModeChips) styleChip(c, absModes shr i and 1 != 0)
        sonarChip.text = if (sonarsOn) "Sonars on" else "Sonars off"
        styleChip(sonarChip, sonarsOn)
        val shown = if (fresh) t.mode.coerceIn(0, 2) else tractionShownMode
        if (shown != tractionShownMode) {
            tractionShownMode = shown
            tractionSeek.progress = tractionByMode[shown]
            tractionLabel.text = tractionText(shown)
        }

        // popup buttons show the current state
        modesBtn.text = if (fresh) "Mode: " + MODE_NAMES.getOrElse(t.mode) { "?" } else "Mode"
        autoBtn.text = if (fresh) "Auto: " + AUTO_NAMES.getOrElse(t.auto) { "?" } else "Auto"
        var assists = (if (stabOn) 1 else 0) + (if (headingOn) 1 else 0)
        for ((_, bit) in assistChips) if (assistMask and bit != 0) assists++
        assistBtn.text = "Assists $assists/${assistChips.size + 2}"
        styleChip(modesBtn, false)
        styleChip(autoBtn, fresh && t.auto != 0)
        styleChip(assistBtn, false, assists > 0)
        styleChip(settingsBtn, false)

        // motion calibration and diagnostics (only drawn while their popup is open, but cheap)
        motionInfo.text = motionStatus(t, fresh)
        diagText.text = if (!fresh) "Not connected" else diagnostics(t)
    }

    private fun diagnostics(t: Telemetry): String {
        val f = t.flags
        val sb = StringBuilder()
        sb.append(String.format(
            Locale.US,
            "Own speed      %4d cm/s\n" +
            "Closing speed  %4d cm/s\n" +
            "Turn rate      %4d dps  (steering gives %d)\n" +
            "Slope          %+5.1f deg\n" +
            "Speed limit    %4d %%\n" +
            "Forward known  %s\n" +
            "Sonars         %s\n" +
            "Sonar C/L/R    %s / %s / %s\n" +
            "G-force        %.1f\n" +
            "Traction       %s\n" +
            "ABS            %s\n" +
            "Speed scale    %.2f cm/s per %%\n" +
            "Braking rate   %4d cm/s2\n" +
            "Active         %s",
            t.vOwn, t.vClose, t.yawAct, t.yawExp, t.slope, t.cap,
            if (t.fwdKnown) "yes" else "no",
            if (t.flags2 and FLAG2_SONAR_ON != 0) "on" else "OFF",
            cm(t.distC), cm(t.distL), cm(t.distR), t.gForce,
            if (t.flags2 and FLAG2_TRACTION != 0) "cutting power" else "idle",
            if (t.flags2 and FLAG2_ABS != 0) "pulsing" else "idle",
            t.kSpeed, t.brakeDecel, flagNames(f)
        ))
        sb.append("\n\nLast calibration run\n")
        if (!t.hasRun) {
            sb.append("none since the car was switched on")
        } else {
            sb.append(String.format(
                Locale.US,
                "Launch push    %4d cm/s2\n" +
                "70-190 cm in   %4d ms\n" +
                "Speed scale    %.2f\n" +
                "Braking rate   %4d cm/s2\n" +
                "Braked hard    %4d ms\n" +
                "Result         %s",
                t.runPush, t.runMs, t.runK, t.runDecel, t.runBrakeMs,
                when (t.calState) {
                    1 -> "running"
                    2 -> "ok"
                    3 -> "no wall 280-400 cm"
                    4, 5 -> "failed, reason ${t.calWhy}"
                    else -> "-"
                }
            ))
        }
        return sb.toString()
    }

    private fun cm(d: Int): String = if (d >= 900) "clear" else "$d"

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
        link.sendQueued("Q,${tractionByMode[mode]}")
        link.sendQueued("A,0")
        link.sendQueued("L,${if (lightsOn) 1 else 0}")
        link.sendLatest("B", "B,${brightSeek.progress}")
        link.sendLatest("R", "R,${rampSeek.progress * 50}")
        link.sendLatest("F", "F,$diffPct")
        link.sendLatest("T", "T,$assistMask")
        link.sendLatest("U", "U,$steerReturn")
        link.sendLatest("J", "J,$gapPct")
        link.sendLatest("Y", "Y,$absModes")
        link.sendLatest("O", "O,${if (sonarEnabled) 1 else 0}")
        for (i in 1..3) link.sendLatest("E$i", "E,$i,${autoMax[i]}")
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
