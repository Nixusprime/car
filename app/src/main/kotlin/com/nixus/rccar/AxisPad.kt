package com.nixus.rccar

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.RectF
import android.view.MotionEvent
import android.view.View
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * A one-axis touch pad with a floating zero.
 *
 * Wherever the finger lands is zero. Moving along the axis gives -100..100 once the finger is past a small
 * dead zone, and full value at [travelDp] from the landing point. A light tap that slips a few dp does nothing.
 * vertical = true gives a throttle (up = +), false gives a steering bar (right = +).
 *
 * The throttle pad also has a brake gesture: swipe left of the landing point past BRAKE_ON_DP and the pad
 * reports brake = true until the finger comes back past BRAKE_OFF_DP. After a brake the value stays at zero
 * until the finger is lifted or comes back to the zero point, so braking never turns into a lurch forward.
 */
class AxisPad(context: Context, private val vertical: Boolean) : View(context) {

    companion object {
        const val DEAD_DP = 8f
        const val BRAKE_ON_DP = 35f
        const val BRAKE_OFF_DP = 20f
    }

    /** Called whenever the value changes, -100..100. */
    var onValue: ((Int) -> Unit)? = null

    /** Called when a finger lands (true) and just before the value returns to 0 (false). */
    var onGrab: ((Boolean) -> Unit)? = null

    /** Called when the finger lifts, with the last value held. Runs before onValue(0). */
    var onRelease: ((Int) -> Unit)? = null

    /** Throttle pad only: swipe left holds the brake. */
    var onBrake: ((Boolean) -> Unit)? = null

    var accent: Int = 0xFF35C46A.toInt()
    var brakeColor: Int = 0xFFEF4B3F.toInt()
    var reverseColor: Int = 0xFFFF8A2B.toInt()
    var title: String = ""
    var hintMin: String = ""
    var hintMax: String = ""
    var hintSide: String = ""

    /** Finger travel from the landing point to 100 %, in dp. Smaller = more sensitive. */
    var travelDp: Float = 70f

    /** Turn off touch (for example while tilt steering is driving the bar). */
    var interactive: Boolean = true

    var percent: Int = 0
        private set

    var braking: Boolean = false
        private set

    private var pointerId = -1
    private var anchorX = 0f
    private var anchorY = 0f
    private var fingerX = 0f
    private var fingerY = 0f
    private var needZero = false
    private val dm = resources.displayMetrics.density

    private val trackPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF1A1D20.toInt() }
    private val edgePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF2C3034.toInt()
        style = Paint.Style.STROKE
        strokeWidth = 2f * dm
    }
    private val tickPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF2C3034.toInt()
        strokeWidth = 2f * dm
        strokeCap = Paint.Cap.ROUND
    }
    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2.5f * dm
    }
    private val knobPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val numPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF0B0D0E.toInt()
        textAlign = Paint.Align.CENTER
        textSize = 17f * dm
        isFakeBoldText = true
    }
    private val hintPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF6B7076.toInt()
        textAlign = Paint.Align.CENTER
        textSize = 12f * dm
    }
    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF8C9096.toInt()
        textAlign = Paint.Align.CENTER
        textSize = 15f * dm
    }
    private val rect = RectF()

    /** Set the value from outside (tilt steering). */
    fun setExternal(p: Int) {
        if (p == percent) return
        percent = p
        invalidate()
    }

    private fun knobRadius(): Float = 24f * dm
    private fun travelPx(): Float = travelDp * dm
    private fun deadPx(): Float = DEAD_DP * dm

    private fun colorForValue(): Int = when {
        braking -> brakeColor
        vertical && percent < 0 -> reverseColor
        else -> accent
    }

    override fun onDraw(canvas: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        val r = 20f * dm
        rect.set(0f, 0f, w, h)
        trackPaint.color = if (braking) 0xFF2A1613.toInt() else 0xFF1A1D20.toInt()
        canvas.drawRoundRect(rect, r, r, trackPaint)
        rect.set(1f, 1f, w - 1f, h - 1f)
        edgePaint.color = if (braking) brakeColor else 0xFF2C3034.toInt()
        canvas.drawRoundRect(rect, r, r, edgePaint)

        val touching = pointerId != -1
        val tr = travelPx()
        val col = colorForValue()

        // idle and nothing driving it from outside: show the scale and hints
        if (!touching && percent == 0) {
            // idle: scale ticks and hints so it is clear what the pad does
            val n = 6
            for (i in -n..n) {
                val f = i / n.toFloat()
                val len = if (i % 3 == 0) 22f * dm else 12f * dm
                if (vertical) {
                    val y = h / 2f - f * (h / 2f - 34f * dm)
                    canvas.drawLine(w / 2f - len / 2f, y, w / 2f + len / 2f, y, tickPaint)
                } else {
                    val x = w / 2f + f * (w / 2f - 34f * dm)
                    canvas.drawLine(x, h / 2f - len / 2f, x, h / 2f + len / 2f, tickPaint)
                }
            }
            if (title.isNotEmpty()) {
                canvas.drawText(title, w / 2f, h / 2f - (if (vertical) 26f else 30f) * dm, titlePaint)
            }
            if (vertical) {
                canvas.drawText(hintMax, w / 2f, 22f * dm, hintPaint)
                canvas.drawText(hintMin, w / 2f, h - 12f * dm, hintPaint)
                if (hintSide.isNotEmpty()) canvas.drawText(hintSide, w / 2f, h / 2f + 38f * dm, hintPaint)
            } else {
                canvas.drawText(hintMin, 42f * dm, h / 2f + 5f * dm, hintPaint)
                canvas.drawText(hintMax, w - 42f * dm, h / 2f + 5f * dm, hintPaint)
            }
            ringPaint.color = 0xFF3A3F44.toInt()
            canvas.drawCircle(w / 2f, h / 2f, knobRadius() * 0.55f, ringPaint)
            return
        }

        // touched: the zero is where the finger landed
        val zx = if (!touching) w / 2f else if (vertical) anchorX.coerceIn(0f, w) else anchorX
        val zy = if (!touching) h / 2f else if (vertical) anchorY else h / 2f
        // travel guide through the zero point
        if (vertical) canvas.drawLine(zx, zy - tr, zx, zy + tr, tickPaint)
        else canvas.drawLine(zx - tr, zy, zx + tr, zy, tickPaint)

        val v = percent / 100f
        val kx = if (vertical) zx else zx + v * tr
        val ky = if (vertical) zy - v * tr else zy
        fillPaint.color = (col and 0x00FFFFFF) or 0x66000000
        fillPaint.strokeWidth = 14f * dm
        fillPaint.strokeCap = Paint.Cap.ROUND
        canvas.drawLine(zx, zy, kx, ky, fillPaint)

        // brake line to the left of the throttle landing point
        if (vertical) {
            val bx = zx - BRAKE_ON_DP * dm
            ringPaint.color = if (braking) brakeColor else 0x66EF4B3F
            canvas.drawLine(bx, zy - 36f * dm, bx, zy + 36f * dm, ringPaint)
            hintPaint.color = if (braking) brakeColor else 0xFF6B7076.toInt()
            canvas.drawText("brake", bx - 2f * dm, zy - 44f * dm, hintPaint)
            hintPaint.color = 0xFF6B7076.toInt()
        }
        // the zero ring and the knob
        ringPaint.color = col
        canvas.drawCircle(zx, zy, deadPx() + 6f * dm, ringPaint)
        knobPaint.color = col
        canvas.drawCircle(kx, ky, knobRadius(), knobPaint)
        val label = if (braking) "B" else percent.toString()
        canvas.drawText(label, kx, ky + numPaint.textSize / 3f, numPaint)
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (!interactive) return false
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                pointerId = e.getPointerId(0)
                anchorX = e.getX(0)
                anchorY = e.getY(0)
                fingerX = anchorX
                fingerY = anchorY
                needZero = false
                parent?.requestDisallowInterceptTouchEvent(true)
                onGrab?.invoke(true)
                update(e, 0)
                invalidate()
            }
            MotionEvent.ACTION_MOVE -> {
                val i = e.findPointerIndex(pointerId)
                if (i >= 0) update(e, i)
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> release()
            MotionEvent.ACTION_POINTER_UP -> {
                if (e.getPointerId(e.actionIndex) == pointerId) release()
            }
        }
        return true
    }

    private fun update(e: MotionEvent, index: Int) {
        fingerX = e.getX(index)
        fingerY = e.getY(index)
        val dead = deadPx()
        val full = travelPx()
        if (full <= dead) return

        // brake gesture (throttle pad only): swipe left of the landing point
        if (vertical) {
            val dx = fingerX - anchorX
            if (!braking && dx < -BRAKE_ON_DP * dm) {
                braking = true
                needZero = true
                onBrake?.invoke(true)
            } else if (braking && dx > -BRAKE_OFF_DP * dm) {
                braking = false
                onBrake?.invoke(false)
            }
        }

        val delta = if (vertical) anchorY - fingerY else fingerX - anchorX      // + = forward / right
        var v = 0f
        if (abs(delta) > dead) {
            v = ((abs(delta) - dead) / (full - dead)).coerceIn(0f, 1f) * (if (delta > 0) 1f else -1f)
        }
        if (needZero) {
            // after a brake the throttle stays at zero until the finger is back at the zero point
            if (abs(delta) <= dead) needZero = false
            v = 0f
        }
        if (braking) v = 0f
        val p = (v * 100f).roundToInt()
        if (p != percent) {
            percent = p
            onValue?.invoke(p)
        }
        invalidate()
    }

    private fun release() {
        if (pointerId == -1) return
        pointerId = -1
        val last = percent
        if (braking) {
            braking = false
            onBrake?.invoke(false)
        }
        needZero = false
        onGrab?.invoke(false)
        onRelease?.invoke(last)
        percent = 0
        onValue?.invoke(0)
        invalidate()
    }
}
