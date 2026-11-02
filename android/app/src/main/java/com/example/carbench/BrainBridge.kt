package com.example.carbench

import android.app.Activity
import android.graphics.Color
import android.os.Handler
import android.os.Looper
import android.view.Gravity
import android.widget.FrameLayout
import android.widget.TextView
import com.chaquo.python.PyObject
import com.chaquo.python.Python
import java.util.concurrent.Executors

object BrainBridge {
    private val io = Executors.newSingleThreadExecutor()      // start/stop fora da UI thread
    private val ui = Handler(Looper.getMainLooper())
    private var statusView: TextView? = null
    private var polling = false

    private fun brain(): PyObject = Python.getInstance().getModule("brain")

    /** Chame no onResume: abre a conexao com o carro (rede CARRINHO). */
    fun start() = io.execute { runCatching { brain().callAttr("start") } }

    /** Chame no onPause: para o carro e fecha a conexao. */
    fun stop() = io.execute { runCatching { brain().callAttr("stop") } }

    /** Chame a cada frame, com o que o StreamRunner devolveu (qualquer tipo). Thread da camera. */
    fun step(resultado: Any?) {
        runCatching { brain().callAttr("step", resultado) }
    }

    fun status(): String =
        runCatching { brain().callAttr("status").toString() }.getOrElse { "brain: erro ${it.message}" }

    /** Chame no onCreate (depois do setContentView): faixa de status no topo da tela. */
    fun attachStatus(act: Activity) {
        val tv = TextView(act).apply {
            setBackgroundColor(Color.argb(170, 0, 0, 0))
            setTextColor(Color.WHITE)
            textSize = 13f
            setPadding(16, 8, 16, 8)
            text = "brain: iniciando"
        }
        act.addContentView(
            tv,
            FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT,
                Gravity.TOP
            )
        )
        statusView = tv
    }

    /** onResume: atualiza a faixa a cada 500 ms. */
    fun resumeStatus() {
        if (polling) return
        polling = true
        ui.post(object : Runnable {
            override fun run() {
                if (!polling) return
                io.execute {
                    val s = status()
                    ui.post { statusView?.text = s }
                }
                ui.postDelayed(this, 500)
            }
        })
    }

    /** onPause. */
    fun pauseStatus() {
        polling = false
    }
}