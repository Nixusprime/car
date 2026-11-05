package com.nixus.rccar

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import kotlin.math.max
import kotlin.math.min

/**
 * Top-down picture of the car with the three sonar readings drawn as beams.
 * The beam length follows the distance, the colour follows how close the obstacle is.
 */
class SonarView(context: Context) : View(context) {

    private var distC = 999
    private var distL = 999
    private var distR = 999
    private var live = false
    private var off = false

    private val dm = resources.displayMetrics.density
    private val rect = RectF()

    private val cardPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF1A1D20.toInt() }
    private val edgePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF2C3034.toInt()
        style = Paint.Style.STROKE
        strokeWidth = 1f * dm
    }
    private val bodyPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF3A4048.toInt() }
    private val wheelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF0E1012.toInt() }
    private val beamPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { strokeWidth = 5f * dm }
    private val dotPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textAlign = Paint.Align.CENTER
        textSize = 12f * dm
    }

    /** Tint the outline: 0 = normal, otherwise the colour of the active warning (braking, speed limited...). */
    fun setEdge(color: Int) {
        if (color == edgeColor) return
        edgeColor = color
        invalidate()
    }

    private var edgeColor = 0

    /** The sonars are switched off in the car: show short grey beams marked "off" instead of "clear". */
    fun setOff(isOff: Boolean) {
        if (isOff == off) return
        off = isOff
        invalidate()
    }

    fun setData(c: Int, l: Int, r: Int, isLive: Boolean) {
        if (c == distC && l == distL && r == distR && isLive == live) return
        distC = c
        distL = l
        distR = r
        live = isLive
        invalidate()
    }

    private fun colorFor(d: Int): Int = when {
        d < 20 -> 0xFFEF4B3F.toInt()
        d < 40 -> 0xFFF2C230.toInt()
        else -> 0xFF35C46A.toInt()
    }

    override fun onDraw(canvas: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        if (w <= 0f || h <= 0f) return
        rect.set(0f, 0f, w, h)
        canvas.drawRoundRect(rect, 14f * dm, 14f * dm, cardPaint)
        rect.set(1f, 1f, w - 1f, h - 1f)
        edgePaint.color = if (edgeColor != 0) edgeColor else 0xFF2C3034.toInt()
        edgePaint.strokeWidth = (if (edgeColor != 0) 2.5f else 1f) * dm
        canvas.drawRoundRect(rect, 14f * dm, 14f * dm, edgePaint)

        // the car, nose up, near the bottom
        val carW = min(w * 0.16f, 34f * dm)
        val carH = min(carW * 1.9f, h * 0.38f)
        val cx = w / 2f
        val carBottom = h - 10f * dm
        val carTop = carBottom - carH
        val wheelW = carW * 0.22f
        val wheelH = carH * 0.24f
        rect.set(cx - carW / 2f - wheelW * 0.5f, carTop + carH * 0.08f, cx - carW / 2f + wheelW * 0.5f, carTop + carH * 0.08f + wheelH)
        canvas.drawRoundRect(rect, 3f * dm, 3f * dm, wheelPaint)
        rect.set(cx + carW / 2f - wheelW * 0.5f, carTop + carH * 0.08f, cx + carW / 2f + wheelW * 0.5f, carTop + carH * 0.08f + wheelH)
        canvas.drawRoundRect(rect, 3f * dm, 3f * dm, wheelPaint)
        rect.set(cx - carW / 2f - wheelW * 0.5f, carBottom - carH * 0.08f - wheelH, cx - carW / 2f + wheelW * 0.5f, carBottom - carH * 0.08f)
        canvas.drawRoundRect(rect, 3f * dm, 3f * dm, wheelPaint)
        rect.set(cx + carW / 2f - wheelW * 0.5f, carBottom - carH * 0.08f - wheelH, cx + carW / 2f + wheelW * 0.5f, carBottom - carH * 0.08f)
        canvas.drawRoundRect(rect, 3f * dm, 3f * dm, wheelPaint)
        rect.set(cx - carW / 2f, carTop, cx + carW / 2f, carBottom)
        canvas.drawRoundRect(rect, 7f * dm, 7f * dm, bodyPaint)

        // beams: 0 degrees straight ahead, side sonars angled 40 degrees outward
        val labelRoom = 18f * dm
        val maxAhead = max(10f * dm, carTop - labelRoom - 4f * dm)
        beam(canvas, cx, carTop, 0f, 1f, distC, maxAhead, w)
        val sideX = carW * 0.32f
        val sideY = carTop + carH * 0.12f
        val sideAhead = max(10f * dm, sideY - labelRoom - 4f * dm)
        beam(canvas, cx - sideX, sideY, -1f, 0.766f, distL, sideAhead, w)
        beam(canvas, cx + sideX, sideY, 1f, 0.766f, distR, sideAhead, w)
    }

    /**
     * dirX is -1, 0 or 1 (which way the beam leans), cosA the vertical share of a beam that leans 40 degrees.
     * maxAhead is how far up the screen the beam may go.
     */
    private fun beam(canvas: Canvas, x0: Float, y0: Float, dirX: Float, cosA: Float, dist: Int, maxAhead: Float, w: Float) {
        val sinA = if (dirX == 0f) 0f else 0.643f
        val clear = dist >= 900
        val frac = if (!live || off) 0.35f else if (clear) 1f else min(1f, max(0.08f, dist / 150f))
        var len = frac * maxAhead / cosA
        if (dirX != 0f) {
            // keep the tip on screen sideways
            val sideRoom = if (dirX < 0f) x0 - 30f * dm else w - x0 - 30f * dm
            len = min(len, max(8f * dm, sideRoom) / sinA)
        }
        val x1 = x0 + dirX * sinA * len
        val y1 = y0 - cosA * len
        val color = if (!live || off) 0xFF3A4048.toInt() else colorFor(dist)
        beamPaint.color = color
        canvas.drawLine(x0, y0, x1, y1, beamPaint)
        dotPaint.color = color
        canvas.drawCircle(x1, y1, 5f * dm, dotPaint)
        textPaint.color = if (live && !off) color else 0xFF8C9096.toInt()
        val label = if (!live) "-" else if (off) "off" else if (clear) "clear" else "$dist cm"
        val lx = min(max(x1, 30f * dm), w - 30f * dm)
        canvas.drawText(label, lx, y1 - 9f * dm, textPaint)
    }
}
