package com.nixus.rccar

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.os.SystemClock
import java.io.InputStream
import java.io.OutputStream
import java.util.LinkedHashMap
import java.util.UUID
import java.util.concurrent.TimeUnit
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

/** Everything the car reports back. */
data class Telemetry(
    val distC: Int = 999,
    val distL: Int = 999,
    val distR: Int = 999,
    val mode: Int = 1,
    val auto: Int = 0,
    val flags: Int = 0,
    val roll: Int = 0,
    val pitch: Int = 0,
    val yaw: Int = 0,
    val gForce: Float = 1f,
    val battV: Float = 0f,
    val speed: Int = 0,
    // motion line (M): what the car works out from its gyro and sonar
    val vOwn: Int = 0,          // own speed, cm/s
    val vClose: Int = 0,        // closing speed on what is ahead, cm/s
    val yawAct: Int = 0,        // measured turn rate, deg/s
    val yawExp: Int = 0,        // turn rate the steering should give, deg/s
    val slope: Float = 0f,      // + = nose up, degrees
    val cap: Int = 100,         // speed limit set by the collision system, %
    val calState: Int = 0,      // calibration: 0 idle, 1 running, 2 done, 3 no wall 280-400 cm, 4 aborted, 5 failed (see calWhy)
    val fwdKnown: Boolean = false,
    val calWhy: Int = 0,        // why the last calibration failed, 1..7 (see the firmware header)
    val brakeDecel: Int = 0,    // braking rate the car uses, cm/s2 (measured by the calibration)
    val kSpeed: Float = 0f,     // real speed in cm/s per % of throttle
    val flags2: Int = 0,        // 1 traction working, 2 ABS pulsing, 4 steering around, 8 sonars on, 16 braking for a roll or push
    // raw numbers of the last calibration run (W line), shown in Diagnostics, also when the run failed
    val hasRun: Boolean = false,
    val runPush: Int = 0,       // mean push felt at launch, cm/s2
    val runMs: Int = 0,         // time for the 70..190 cm stretch, ms
    val runK: Float = 0f,       // measured cm/s per % of throttle
    val runDecel: Int = 0,      // measured braking rate, cm/s2
    val runBrakeMs: Int = 0     // how long the car braked hard, ms
)

/**
 * Low-latency Bluetooth link to the car (classic SPP, HC-05 / HC-06).
 *
 * Design for no perceived delay:
 *  - drive values are a single "latest" snapshot, never a queue, so old stick positions are dropped
 *  - a dedicated sender thread wakes the instant a value changes and sends at most every MIN_GAP_MS
 *  - when nothing changes it repeats the drive line every HEARTBEAT_MS, which also feeds the car's failsafe
 *  - one-shot commands (mode, lights, ...) are sent in order, just before the next drive line
 *  - slider-style commands are coalesced by key, so only the newest value is ever sent
 */
class CarLink(private val listener: Listener) {

    interface Listener {
        fun onState(state: State, message: String)
        fun onTelemetry(t: Telemetry)
    }

    enum class State { DISCONNECTED, CONNECTING, CONNECTED }

    companion object {
        private const val MIN_GAP_MS = 25L
        private const val HEARTBEAT_MS = 100L
        private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    }

    @Volatile private var session = 0
    @Volatile private var socket: BluetoothSocket? = null
    @Volatile private var throttle = 0
    @Volatile private var steer = 0
    @Volatile private var brake = false

    private val lock = ReentrantLock()
    private val wake = lock.newCondition()
    private var dirty = false
    private val pending = LinkedHashMap<String, String>()
    private var counter = 0

    val isConnected: Boolean get() = socket != null

    /** Latest stick positions, -100..100, and whether the full brake is held. Cheap to call as often as you like. */
    fun setDrive(t: Int, s: Int, b: Boolean = false) {
        if (t == throttle && s == steer && b == brake) return
        throttle = t
        steer = s
        brake = b
        lock.withLock {
            dirty = true
            wake.signalAll()
        }
    }

    /** A command that must be delivered in order (mode change, lights, brake...). */
    fun sendQueued(cmd: String) {
        counter++
        put("#$counter", cmd)
    }

    /** A command where only the newest value matters (sliders). */
    fun sendLatest(key: String, cmd: String) {
        put(key, cmd)
    }

    private fun put(key: String, cmd: String) {
        if (socket == null) return
        lock.withLock {
            if (pending.size > 64) {
                val first = pending.keys.iterator().next()
                pending.remove(first)
            }
            pending[key] = cmd
            dirty = true
            wake.signalAll()
        }
    }

    /** Best effort: tell the car to stop right now, even from the UI thread. */
    fun emergencyStopNow() {
        throttle = 0
        brake = false
        try {
            val out = socket?.outputStream
            if (out != null) {
                out.write("X\nD,0,$steer,0\n".toByteArray(Charsets.US_ASCII))
                out.flush()
            }
        } catch (e: Exception) {
            // the sender thread will notice a dead link on its own
        }
        lock.withLock {
            dirty = true
            wake.signalAll()
        }
    }

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        disconnect()
        val my = ++session
        listener.onState(State.CONNECTING, device.name ?: device.address)
        Thread {
            var s: BluetoothSocket? = null
            try {
                try {
                    s = device.createRfcommSocketToServiceRecord(SPP_UUID)
                    s.connect()
                } catch (first: Exception) {
                    try { s?.close() } catch (ignored: Exception) { }
                    // some HC-05 clones only answer the fixed-channel socket
                    val m = device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                    s = m.invoke(device, 1) as BluetoothSocket
                    s.connect()
                }
            } catch (e: Exception) {
                try { s?.close() } catch (ignored: Exception) { }
                if (my == session) listener.onState(State.DISCONNECTED, "Couldn't connect. Is the car on and paired?")
                return@Thread
            }
            if (my != session) {
                try { s?.close() } catch (ignored: Exception) { }
                return@Thread
            }
            val sock: BluetoothSocket = s ?: return@Thread
            socket = sock
            lock.withLock { pending.clear() }
            listener.onState(State.CONNECTED, device.name ?: device.address)
            try {
                val input = sock.inputStream
                val output = sock.outputStream
                Thread { readLoop(my, input) }.start()
                sendLoop(my, output)
            } catch (e: Exception) {
                fail(my, "Connection lost")
            }
        }.start()
    }

    fun disconnect() {
        session++
        val s = socket
        socket = null
        try { s?.close() } catch (e: Exception) { }
        lock.withLock {
            pending.clear()
            wake.signalAll()
        }
    }

    private fun fail(my: Int, message: String) {
        if (my != session) return
        session++
        val s = socket
        socket = null
        try { s?.close() } catch (e: Exception) { }
        lock.withLock { wake.signalAll() }
        listener.onState(State.DISCONNECTED, message)
    }

    private fun sendLoop(my: Int, out: OutputStream) {
        var lastSend = 0L
        try {
            while (my == session) {
                var cmds: List<String> = emptyList()
                lock.withLock {
                    if (!dirty) wake.await(HEARTBEAT_MS, TimeUnit.MILLISECONDS)
                    dirty = false
                    if (pending.isNotEmpty()) {
                        cmds = ArrayList(pending.values)
                        pending.clear()
                    }
                }
                if (my != session) break
                val gap = MIN_GAP_MS - (SystemClock.elapsedRealtime() - lastSend)
                if (gap > 0) Thread.sleep(gap)
                val sb = StringBuilder()
                for (c in cmds) sb.append(c).append('\n')
                sb.append("D,").append(throttle).append(',').append(steer).append(',').append(if (brake) 1 else 0).append('\n')
                out.write(sb.toString().toByteArray(Charsets.US_ASCII))
                out.flush()
                lastSend = SystemClock.elapsedRealtime()
            }
        } catch (e: Exception) {
            fail(my, "Connection lost")
        }
    }

    private fun readLoop(my: Int, input: InputStream) {
        val buf = ByteArray(256)
        val line = StringBuilder()
        var t = Telemetry()
        try {
            while (my == session) {
                val n = input.read(buf)
                if (n < 0) break
                for (i in 0 until n) {
                    val c = (buf[i].toInt() and 0xFF).toChar()
                    if (c == '\n') {
                        val parsed = parse(line.toString().trim(), t)
                        line.setLength(0)
                        if (parsed != null) {
                            t = parsed
                            if (my == session) listener.onTelemetry(t)
                        }
                    } else if (c != '\r' && line.length < 100) {
                        line.append(c)
                    }
                }
            }
        } catch (e: Exception) {
            // handled below
        }
        fail(my, "Connection lost")
    }

    private fun parse(l: String, prev: Telemetry): Telemetry? {
        val p = l.split(',')
        try {
            if (p.size >= 7 && p[0] == "S") {
                return prev.copy(
                    distC = p[1].toInt(), distL = p[2].toInt(), distR = p[3].toInt(),
                    mode = p[4].toInt(), auto = p[5].toInt(), flags = p[6].toInt()
                )
            }
            if (p.size >= 7 && p[0] == "I") {
                return prev.copy(
                    roll = p[1].toInt(), pitch = p[2].toInt(), yaw = p[3].toInt(),
                    gForce = p[4].toInt() / 10f, battV = p[5].toInt() / 100f, speed = p[6].toInt()
                )
            }
            if (p.size >= 6 && p[0] == "W") {
                return prev.copy(
                    hasRun = true, runPush = p[1].toInt(), runMs = p[2].toInt(), runK = p[3].toInt() / 100f,
                    runDecel = p[4].toInt(), runBrakeMs = p[5].trim().toInt()
                )
            }
            if (p.size >= 13 && p[0] == "M") {
                return prev.copy(
                    vOwn = p[1].toInt(), vClose = p[2].toInt(), yawAct = p[3].toInt(), yawExp = p[4].toInt(),
                    slope = p[5].toInt() / 10f, cap = p[6].toInt(), calState = p[7].toInt(), fwdKnown = p[8].trim() != "0",
                    calWhy = p[9].toInt(), brakeDecel = p[10].toInt(), kSpeed = p[11].toInt() / 100f, flags2 = p[12].trim().toInt()
                )
            }
        } catch (e: NumberFormatException) {
            // a garbled line: ignore it
        }
        return null
    }
}
