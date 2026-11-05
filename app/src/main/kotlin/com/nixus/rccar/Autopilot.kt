package com.nixus.rccar

import kotlin.math.max
import kotlin.math.min

/**
 * Roam, Follow and Corridor. These used to run inside the Arduino sketch; they now run on the phone to free flash on the
 * Nano. The car sends the sonar distances about ten times a second, this class turns them into a throttle and steering
 * value, and the app sends them back as the normal D line. The car's own collision system, failsafe and stuck cut-off
 * still sit on top, and if the link drops the car stops by itself.
 */
class Autopilot {

    companion object {
        private const val GO = 0
        private const val BACK = 1
        private const val TURN = 2
        private const val BACK_PCT = 25f
        private const val TURN_PCT = 35f
        private const val BACK_MS = 450L
        private const val TURN_MS = 600L
        private const val FOLLOW_TARGET = 30f
        private const val FOLLOW_LOSE = 90f
        private const val FOLLOW_DEADBAND = 5f
        private const val BRAKE_BASE = 15f
        private const val BRAKE_PER_PCT = 0.30f
        private const val HALF_WIDTH = 7f
        private const val SIDE_MARGIN = 3f
        private const val SIDE_SIN = 0.64f
        private const val SIDE_COS = 0.77f
    }

    var throttle = 0
        private set
    var steer = 0
        private set
    var gaveUp = false
        private set

    private var lastMode = 0
    private var state = GO
    private var dir = 1
    private var until = 0L
    private var backCount = 0
    private var backWindow = 0L

    fun clearGaveUp() {
        gaveUp = false
    }

    private fun sideHit(d: Float, thr: Float): Boolean {
        if (d >= 900f) return false
        return d * SIDE_SIN < HALF_WIDTH + SIDE_MARGIN && d * SIDE_COS < thr
    }

    private fun blocked(t: Telemetry, pct: Float): Boolean {
        val thr = BRAKE_BASE + pct * BRAKE_PER_PCT
        return t.distC < thr || sideHit(t.distL.toFloat(), thr) || sideHit(t.distR.toFloat(), thr)
    }

    private fun escape(now: Long, dC: Float, dL: Float, dR: Float) {
        dir = if (dR >= dL) 1 else -1
        if (dC > 18f && max(dL, dR) > 35f) {
            state = TURN
            until = now + TURN_MS
            return
        }
        if (now - backWindow > 8000L) {
            backWindow = now
            backCount = 0
        }
        if (backCount >= 2) {
            gaveUp = true
            return
        }
        backCount++
        state = BACK
        until = now + BACK_MS
    }

    /** Work out the next throttle and steering for [mode] (1 Roam, 2 Follow, 3 Corridor), top speed [topPct] in %. */
    fun step(t: Telemetry, mode: Int, topPct: Int, now: Long) {
        if (mode != lastMode) {
            lastMode = mode
            state = GO
            backCount = 0
            gaveUp = false
        }
        val dC = t.distC.toFloat()
        val dL = t.distL.toFloat()
        val dR = t.distR.toFloat()
        val top = topPct.toFloat()
        var th = 0f
        var st = 0f
        when (mode) {
            1 -> when (state) {
                GO -> {
                    if (blocked(t, top)) {
                        escape(now, dC, dL, dR)
                    } else {
                        val slow = ((dC - 25f) / 55f).coerceIn(0.5f, 1f)
                        th = top * slow
                        st = ((min(dR, 100f) - min(dL, 100f)) * 0.9f).coerceIn(-100f, 100f)
                    }
                }
                BACK -> {
                    th = -BACK_PCT
                    st = -dir * 100f
                    if (now >= until || t.gForce > 1.5f) {
                        state = TURN
                        until = now + TURN_MS
                    }
                }
                else -> {
                    th = TURN_PCT
                    st = dir * 100f
                    if (blocked(t, TURN_PCT)) escape(now, dC, dL, dR)
                    else if (now >= until) state = GO
                }
            }
            2 -> if (dC <= FOLLOW_LOSE) {
                val err = dC - FOLLOW_TARGET
                if (err > FOLLOW_DEADBAND) th = (err * 2f).coerceIn(0f, top)
                else if (err < -12f) th = (err * 0.8f).coerceIn(-10f, 0f)
                st = ((min(dL, 100f) - min(dR, 100f)) * 1.2f).coerceIn(-100f, 100f)
            }
            3 -> {
                th = top
                st = ((min(dR, 150f) - min(dL, 150f)) * 0.8f).coerceIn(-100f, 100f)
            }
        }
        throttle = th.toInt().coerceIn(-100, 100)
        steer = st.toInt().coerceIn(-100, 100)
    }
}
