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
 * A one-axis touch pad that springs back to zero when released.
 * vertical = true gives a throttle (up = +), false gives a steering bar (right = +).
 */
class AxisPad(context: Context, private val vertical: Boolean) : View(context) {

    /** Called whenever the value changes, -100..100. */
    var onValue: ((Int) -> Unit)? = null

    /** Called when a finger lands (true) and just before the value returns to 0 (false). */
    var onGrab: ((Boolean) -> Unit)? = null

    /** Called when the finger lifts, with the last value held. Runs before onValue(0). */
    var onRelease: ((Int) -> Unit)? = null

    var accent: Int = 0xFF3B82F6.toInt()

    /** Turn off touch (for example while tilt steering is driving the bar). */
    var interactive: Boolean = true

    var percent: Int = 0
        private set

    private var pointerId = -1
    private val dm = resources.displayMetrics.density

    private val trackPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF171B22.toInt() }
    private val edgePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF2A303B.toInt()
        style = Paint.Style.STROKE
        strokeWidth = 2f * dm
    }
    private val midPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF2A303B.toInt()
        strokeWidth = 2f * dm
    }
    private val knobPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFFFFFFFF.toInt()
        textAlign = Paint.Align.CENTER
        textSize = 16f * dm
    }
    private val rect = RectF()

    /** Set the value from outside (tilt steering). */
    fun setExternal(p: Int) {
        if (p == percent) return
        percent = p
        invalidate()
    }

    private fun knobRadius(): Float = (if (vertical) width else height) * 0.32f

    private fun travel(): Float {
        val half = if (vertical) height / 2f else width / 2f
        return half - knobRadius() - 8f * dm
    }

    override fun onDraw(canvas: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        val r = 20f * dm
        rect.set(0f, 0f, w, h)
        canvas.drawRoundRect(rect, r, r, trackPaint)
        rect.set(1f, 1f, w - 1f, h - 1f)
        canvas.drawRoundRect(rect, r, r, edgePaint)
        if (vertical) canvas.drawLine(w * 0.2f, h / 2f, w * 0.8f, h / 2f, midPaint)
        else canvas.drawLine(w / 2f, h * 0.2f, w / 2f, h * 0.8f, midPaint)

        val k = knobRadius()
        val tr = travel()
        val cx = if (vertical) w / 2f else w / 2f + percent / 100f * tr
        val cy = if (vertical) h / 2f - percent / 100f * tr else h / 2f
        knobPaint.color = if (vertical && percent < 0) 0xFFF59E0B.toInt() else accent
        canvas.drawCircle(cx, cy, k, knobPaint)
        canvas.drawText(percent.toString(), cx, cy + textPaint.textSize / 3f, textPaint)
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (!interactive) return false
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                pointerId = e.getPointerId(0)
                parent?.requestDisallowInterceptTouchEvent(true)
                onGrab?.invoke(true)
                update(e, 0)
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
        val tr = travel()
        if (tr <= 0f) return
        val raw = if (vertical) (height / 2f - e.getY(index)) / tr else (e.getX(index) - width / 2f) / tr
        var v = raw.coerceIn(-1f, 1f)
        if (abs(v) < 0.05f) v = 0f
        val p = (v * 100f).roundToInt()
        if (p != percent) {
            percent = p
            onValue?.invoke(p)
            invalidate()
        }
    }

    private fun release() {
        if (pointerId == -1) return
        pointerId = -1
        val last = percent
        onGrab?.invoke(false)
        onRelease?.invoke(last)
        percent = 0
        onValue?.invoke(0)
        invalidate()
    }
}
