plugins {
    alias(libs.plugins.android.application)
    id("com.chaquo.python")
}

android {
    namespace = "com.example.carbench"
    compileSdk {
        version = release(37)
    }

    defaultConfig {
        applicationId = "com.example.carbench"
        minSdk = 24
        targetSdk = 37
        versionCode = 1
        versionName = "1.0"

        ndk {
            // S9 = arm64. Acrescente "x86_64" se for usar o emulador do Android Studio.
            abiFilters += listOf("arm64-v8a")
        }

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            optimization {
                enable = true
                packageScope = setOf("androidx.**", "kotlin.**", "kotlinx.**")
            }
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}

dependencies {
    implementation(libs.androidx.activity.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.androidx.constraintlayout)
    implementation(libs.androidx.core.ktx)
    implementation(libs.material)
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(libs.androidx.junit)
    val camerax = "1.3.4"
    implementation("androidx.camera:camera-core:$camerax")
    implementation("androidx.camera:camera-camera2:$camerax")
    implementation("androidx.camera:camera-lifecycle:$camerax")
    implementation("androidx.camera:camera-view:$camerax")
    implementation("androidx.activity:activity-ktx:1.8.2")
}

chaquopy {
    defaultConfig {
        version = "3.10"
        // Só se o Chaquopy não achar o Python 3.10 do PC sozinho:
        // buildPython("C:/Users/SEU_USUARIO/AppData/Local/Programs/Python/Python310/python.exe")
        pip {
            // manter igual a requirements-phone.txt
            install("numpy==1.26.2")
            install("opencv-python-headless==4.5.1.48")
        }
    }
}

// Copia o código do repositório (uma única fonte da verdade) para a pasta Python do app
// a cada build. NAO coloque outros arquivos em app/src/main/python: o Sync apaga o que sobra.
val syncPython = tasks.register<Sync>("syncPython") {
    from(rootDir.parentFile) {          // rootDir = <repo>/android  ->  parentFile = <repo>
        include("config.py", "vision/*.py", "control/*.py",
            "tools/__init__.py", "tools/bench_s9.py", "tools/android_entry.py", "main.py")
    }
    into(layout.projectDirectory.dir("src/main/python"))
}
tasks.matching { it.name == "preBuild" || (it.name.contains("Python") && it.name != "syncPython") }.configureEach {
    dependsOn(syncPython)
}