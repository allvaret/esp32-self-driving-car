package com.example.carbench

import android.os.Bundle
import android.widget.Button
import android.widget.CheckBox
import android.widget.EditText
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.chaquo.python.Python
import java.io.File
import kotlin.concurrent.thread

/**
 * Esqueleto minimo: dois botoes chamam funcoes Python (tools/android_entry.py).
 *  1) "Teste de imports"  -> confirma Python/numpy/OpenCV e o que o OpenCV sabe ler de video.
 *  2) "Rodar bench"       -> roda o pipeline sobre assets/bench.mp4 (ou assets/frames/'*'.jpg).
 * O Python ja foi iniciado pela PyApplication (declarada no AndroidManifest).
*/



class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val out = findViewById<TextView>(R.id.out)
        val btnSmoke = findViewById<Button>(R.id.btnSmoke)
        val btnBench = findViewById<Button>(R.id.btnBench)
        val chkEdges = findViewById<CheckBox>(R.id.chkEdges)
        val chkFrames = findViewById<CheckBox>(R.id.chkFrames)
        val edtThreads = findViewById<EditText>(R.id.edtThreads)
        val buttons = listOf(btnSmoke, btnBench)

        btnSmoke.setOnClickListener {
            runPython(out, buttons) { py ->
                py.getModule("tools.android_entry").callAttr("smoke").toString()
            }
        }

        btnBench.setOnClickListener {
            val threads = edtThreads.text.toString().toIntOrNull() ?: 0
            val edges = chkEdges.isChecked
            val useFrames = chkFrames.isChecked
            runPython(out, buttons) { py ->
                val path = if (useFrames) copyAssetDir("frames").absolutePath
                    else copyAssetFile("bench.mp4").absolutePath
                py.getModule("tools.android_entry")
                    .callAttr("bench", path, threads, edges, 150).toString()
            }
        }
    }

/** Roda 'block' fora da thread de UI (senao o app congela) e mostra o resultado no texto. */
    private fun runPython(out: TextView, buttons: List<Button>, block: (Python) -> String) {
        buttons.forEach { it.isEnabled = false }
        out.text = "Rodando... (a 1a chamada importa numpy/cv2 e demora mais)"
        thread {
            val text = try {
                block(Python.getInstance())
            } catch (e: Throwable) {
                "ERRO: $e"
            }
            runOnUiThread {
                out.text = text
                buttons.forEach { it.isEnabled = true }
            }
        }
    }

/** Copia um arquivo de assets/ para o cache (o OpenCV precisa de um caminho de arquivo). */
    private fun copyAssetFile(name: String): File {
        val dst = File(cacheDir, name)
        assets.open(name).use { i -> dst.outputStream().use { o -> i.copyTo(o) } }
        return dst
    }

/** Copia uma pasta de assets/ (ex.: frames/) para o cache, apagando a copia anterior. */
    private fun copyAssetDir(name: String): File {
        val dst = File(cacheDir, name).apply { deleteRecursively(); mkdirs() }
        for (f in assets.list(name) ?: emptyArray()) {
            assets.open("$name/$f").use { i -> File(dst, f).outputStream().use { o -> i.copyTo(o) } }
        }
        return dst
    }
}