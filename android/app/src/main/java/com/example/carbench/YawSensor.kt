package com.example.carbench

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.HandlerThread
import com.chaquo.python.Python
import kotlin.math.sqrt

/**
 * Giro em torno da VERTICAL (graus/s, > 0 = virando para a DIREITA) para o brain.on_gyro.
 * Projeta o giroscopio na gravidade, entao nao importa como o celular esta preso no carro.
 * Roda numa thread propria (nao na UI) e chama o Python a ~50 Hz.
 */
class YawSensor(ctx: Context) : SensorEventListener {
    private val sm = ctx.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private var thread: HandlerThread? = null   // uma thread nova a cada start() (nao reinicia depois de quit)
    private val g = FloatArray(3)          // direcao "para cima" no referencial do celular
    private var haveG = false
    private var running = false

    fun start() {
        if (running) return
        val gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE) ?: return
        // TYPE_GRAVITY quando existe; senao o acelerometro (com o carro andando fica ruidoso)
        val grav = sm.getDefaultSensor(Sensor.TYPE_GRAVITY) ?: sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val th = HandlerThread("yaw-sensor").also { it.start(); thread = it }
        val h = android.os.Handler(th.looper)
        sm.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME, h)
        if (grav != null) sm.registerListener(this, grav, SensorManager.SENSOR_DELAY_GAME, h)
        running = true
    }

    fun stop() {
        if (!running) return
        sm.unregisterListener(this)
        thread?.quitSafely()
        thread = null
        running = false
    }

    override fun onSensorChanged(e: SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_GRAVITY, Sensor.TYPE_ACCELEROMETER -> {
                val a = 0.1f   // passa-baixa leve (no TYPE_GRAVITY ja vem limpo)
                for (i in 0..2) g[i] += a * (e.values[i] - g[i])
                haveG = true
            }
            Sensor.TYPE_GYROSCOPE -> {
                if (!haveG) return
                val n = sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
                if (n < 1f) return
                // omega . up > 0 = anti-horario visto de cima = esquerda  ->  direita = menos
                val wUp = (e.values[0] * g[0] + e.values[1] * g[1] + e.values[2] * g[2]) / n
                val rightDps = -Math.toDegrees(wUp.toDouble())
                runCatching { Python.getInstance().getModule("brain").callAttr("on_gyro", rightDps) }
            }
        }
    }

    override fun onAccuracyChanged(s: Sensor?, a: Int) {}
}
