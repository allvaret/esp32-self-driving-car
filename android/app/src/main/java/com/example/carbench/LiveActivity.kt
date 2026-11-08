package com.example.carbench 

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Bundle
import android.util.Size
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.core.*
import androidx.camera.core.resolutionselector.AspectRatioStrategy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import com.chaquo.python.PyObject
import com.chaquo.python.Python
import com.chaquo.python.android.AndroidPlatform
import org.json.JSONObject
import java.io.File
import java.util.concurrent.Executors

class LiveActivity : ComponentActivity() {
    private lateinit var previewView: PreviewView
    private lateinit var txtState: TextView
    private lateinit var txtInfo: TextView
    private val exec = Executors.newSingleThreadExecutor()   // TODO o Python roda AQUI
    private var stream: PyObject? = null
    private var buf: ByteArray? = null
    @Volatile private var wantCal = false
    private var lastNs = 0L
    private var fps = 0.0

    private val askCam = registerForActivityResult(ActivityResultContracts.RequestPermission()) { ok ->
        if (ok) startCamera() else txtState.text = "SEM PERMISSAO"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_live)
        BrainBridge.attachStatus(this)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        previewView = findViewById(R.id.preview)
        previewView.scaleType = PreviewView.ScaleType.FIT_CENTER   // mostra o mesmo campo que a analise
        txtState = findViewById(R.id.txtState)
        txtInfo = findViewById(R.id.txtInfo)
        findViewById<Button>(R.id.btnCal).setOnClickListener { wantCal = true }
        if (!Python.isStarted()) Python.start(AndroidPlatform(this))

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            == PackageManager.PERMISSION_GRANTED) startCamera()
        else askCam.launch(Manifest.permission.CAMERA)
    }

    private fun startCamera() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            val provider = future.get()
            val sel = ResolutionSelector.Builder()
                .setAspectRatioStrategy(AspectRatioStrategy.RATIO_16_9_FALLBACK_AUTO_STRATEGY)
                .setResolutionStrategy(ResolutionStrategy(Size(640, 360),
                    ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
                .build()
            val preview = Preview.Builder().setResolutionSelector(sel).build()
                .also { it.setSurfaceProvider(previewView.surfaceProvider) }
            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(sel)
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
            analysis.setAnalyzer(exec) { img -> analyze(img) }
            provider.unbindAll()
            provider.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis)
        }, ContextCompat.getMainExecutor(this))
    }

    private fun analyze(img: ImageProxy) {
        try {
            val s = stream ?: Python.getInstance().getModule("main")
                .callAttr("StreamRunner", true, File(filesDir, "live_log.csv").absolutePath)
                .also { stream = it }
            if (wantCal) { wantCal = false; s.callAttr("recalibrate") }

            val plane = img.planes[0]
            val bb = plane.buffer.also { it.rewind() }
            val n = bb.remaining()
            val arr = buf?.takeIf { it.size == n } ?: ByteArray(n).also { buf = it }
            bb.get(arr)

            val t0 = System.nanoTime()
            val json = s.callAttr("process_rgba_frame", arr, img.width, img.height,
                plane.rowStride, img.imageInfo.rotationDegrees).toString()
            BrainBridge.step(json)
            val pyMs = (System.nanoTime() - t0) / 1e6

            val now = System.nanoTime()
            if (lastNs != 0L) fps = 0.9 * fps + 0.1 * (1e9 / (now - lastNs))
            lastNs = now

            val o = JSONObject(json)
            runOnUiThread { render(o, pyMs) }
        } catch (e: Exception) {
            runOnUiThread { txtState.text = "ERRO KT"; txtInfo.text = e.toString() }
        } finally {
            img.close()
        }
    }

    private fun render(o: JSONObject, pyMs: Double) {
        val phase = o.optString("phase")
        val vs = o.optString("vision_state", "-")
        txtState.text = if (phase == "RUN") vs else phase
        txtState.setTextColor(when {
            phase == "ERROR" -> Color.MAGENTA
            vs.startsWith("LIVRE") -> Color.GREEN
            vs.startsWith("INCERTO") -> Color.YELLOW
            vs.startsWith("OBST") -> Color.RED
            else -> Color.LTGRAY
        })
        txtInfo.text = if (phase == "ERROR") o.optString("why") + "\n" + o.optString("trace")
        else String.format(
            "ctrl=%s %s\n%s\nfps=%.1f  py=%.0fms (conv %.0f, proc %.0f)\n%dx%d rot=%d  r=%s",
            o.optString("ctrl_state"), o.optString("sub"), o.optString("why"),
            fps, pyMs, o.optDouble("conv_ms", 0.0), o.optDouble("proc_ms", 0.0),
            o.optInt("w"), o.optInt("h"), o.optInt("rot"), o.opt("r"))
    }

    override fun onDestroy() {
        exec.execute { stream?.callAttr("close") }   // fecha o CSV na mesma thread do Python
        exec.shutdown()
        super.onDestroy()
    }

    override fun onResume() { super.onResume(); BrainBridge.start(); BrainBridge.resumeStatus() }

    override fun onPause()  { BrainBridge.pauseStatus(); BrainBridge.stop(); super.onPause() }
}
