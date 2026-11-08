# App mínimo com Chaquopy (CarBench)

Objetivo: rodar o `tools/bench_s9.py` **dentro do S9**, com o Python e o OpenCV que
irão para o carro, e ver fps e latência por etapa.

> **Status:** os arquivos Python deste guia foram testados no PC (inclusive em Python 3.10
> com numpy 1.26.2 e OpenCV 4.5.5). **O build Android (Gradle, Chaquopy, Kotlin) não foi
> testado**: não há Android SDK onde escrevi isto. Se algo falhar, a seção
> "Erros comuns" cobre os mais prováveis; me mande a mensagem completa.

Versões usadas (conferidas na documentação do Chaquopy 17.1): plugin `17.1.0`,
Android Gradle Plugin entre 7.3 e 9.4, `minSdk` ≥ 24, Python 3.10, `numpy==1.26.2`,
`opencv-python-headless==4.5.1.48` (únicas versões de OpenCV com wheel Android para o 3.10).

---

## 0. Antes de abrir o Android Studio

1. **Python 3.10 no PC.** O Chaquopy usa um Python do PC *com a mesma versão (maior.menor)
   do app* para rodar o `pip` e compilar o código. Você tem o 3.11.9, então instale também o
   **3.10** (o último instalador binário do 3.10 é o 3.10.11, em python.org). Confira no terminal:
   ```
   py -3.10 --version
   ```
   Deve imprimir `Python 3.10.x`. (O Chaquopy tenta `py -3.10` sozinho no Windows.)
2. **S9 com depuração USB:** Configurações → Sobre o telefone → Informações do software →
   toque 7 vezes em *Número da versão* → volte → *Opções do desenvolvedor* → ligue
   *Depuração USB*. Conecte com um cabo que transfere dados e aceite o aviso
   "Permitir depuração USB?" no celular. Se o S9 não aparecer no Android Studio, instale o
   driver USB da Samsung.
3. **Caminho curto.** O Windows limita caminhos a 260 caracteres e o build do Chaquopy gera
   muitos arquivos. Se o seu repositório estiver numa pasta muito funda, o build pode falhar:
   mova-o para algo como `C:\dev\Esp32_Self_driver_car`.
4. Um vídeo curto (ou frames) para o teste, de preferência o `10sec_reframed10.mp4`.

## 1. Criar o projeto

*File → New → New Project → **Empty Views Activity** → Next*, e preencha:

| Campo | Valor |
|---|---|
| Name | `CarBench` |
| Package name | `com.example.carbench` (os arquivos prontos usam este) |
| Save location | `<repositório>\android` (a pasta `android` dentro do repositório) |
| Language | Kotlin |
| Minimum SDK | **API 24** |
| Build configuration language | Kotlin DSL (`build.gradle.kts`) |

*Finish*. Espere o primeiro "Gradle sync" terminar (barra de progresso embaixo; na primeira
vez baixa bastante coisa).

> Gradle é a ferramenta de build do Android: ela lê os arquivos `build.gradle.kts`,
> baixa as dependências e monta o APK. "Sync" = reler esses arquivos.

## 2. Editar os arquivos do Gradle

No painel **Project** (à esquerda, modo *Android*), os arquivos ficam em *Gradle Scripts*.
Se preferir ver as pastas reais, troque o modo para *Project*.

### 2.1 `android/build.gradle.kts` (o do **projeto**, "Project: CarBench")
Dentro do bloco `plugins { ... }`, **acrescente** a linha:
```kotlin
id("com.chaquo.python") version "17.1.0" apply false
```

### 2.2 `android/app/build.gradle.kts` (o do **módulo**, "Module: app")

**(a)** No bloco `plugins { ... }` acrescente:
```kotlin
id("com.chaquo.python")
```

**(b)** Dentro de `android { defaultConfig { ... } }` confirme `minSdk = 24` e acrescente:
```kotlin
ndk {
    // S9 = arm64. Acrescente "x86_64" se for usar o emulador do Android Studio.
    abiFilters += listOf("arm64-v8a")
}
```

**(c)** No **fim** do arquivo (fora de qualquer bloco), acrescente:
```kotlin
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
val syncPython by tasks.registering(Sync::class) {
    from(rootDir.parentFile) {          // rootDir = <repo>/android  ->  parentFile = <repo>
        include("config.py", "vision/*.py", "control/*.py", "hardware/*.py",
                "tools/__init__.py", "tools/bench_s9.py", "tools/android_entry.py")
    }
    into(layout.projectDirectory.dir("src/main/python"))
}
tasks.matching { it.name == "preBuild" }.configureEach { dependsOn(syncPython) }
```

Depois clique em **Sync Now** (ou no ícone do elefante, *Sync Project with Gradle Files*).

> Confira também a versão do Android Gradle Plugin (arquivo `gradle/libs.versions.toml`,
> linha `agp = "..."`). A documentação do Chaquopy 17.1 declara suporte de 7.3.x a 9.4.x;
> versões mais novas "podem funcionar", mas não foram testadas por eles.

### 2.3 `app/src/main/AndroidManifest.xml`
Na tag `<application ...>` **acrescente** o atributo (isso inicia o Python junto com o app):
```xml
android:name="com.chaquo.python.android.PyApplication"
```

## 3. Colocar os arquivos do app

1. **Kotlin:** substitua o conteúdo de `app/src/main/java/com/example/carbench/MainActivity.kt`
   por `android/app_files/MainActivity.kt`.
2. **Layout:** substitua `app/src/main/res/layout/activity_main.xml` por
   `android/app_files/activity_main.xml`.
3. **Dados de teste:** crie a pasta *assets* (clique direito em `app` → *New → Folder →
   Assets Folder*) e coloque lá **um** destes:
    - `bench.mp4` (o vídeo, com este nome exato), **ou**
    - uma pasta `frames/` com JPEGs, gerada no PC por:
      ```
      python -m tools.video_to_frames 10sec_reframed10.mp4 android/app/src/main/assets/frames
      ```
   Use os frames se o teste 1) mostrar que o OpenCV do celular não tem decodificador de vídeo.
4. Rode **Sync** de novo.

## 4. Rodar

1. No topo, na lista de dispositivos, escolha o seu S9 (aparece com o modelo).
2. Clique no **triângulo verde (Run 'app')**. O primeiro build demora (baixa o Python, instala
   os pacotes via pip, compila). Os seguintes são bem mais rápidos.
3. O app abre no celular. Toque em **1) Teste de imports**. Esperado, algo como:
   ```
   python 3.10.x | numpy 1.26.2 | cv2 4.5.1
   Video I/O do OpenCV:
   ... (FFMPEG / MediaNDK: YES ou NO)
   imencode/imdecode jpg: ok
   ```
4. Toque em **2) Rodar bench**. O resultado é a mesma tabela do PC (média, p95, máximo).
   O texto é selecionável: copie e me envie.

### O que testar no S9
Rode 3 vezes cada e anote a mediana: threads **0** (padrão), **1**, **2** e **4**; com e sem
bordas. A 1ª execução inclui o import de numpy/cv2 (mais lenta), então descarte-a.
Deixe o celular carregado e sem outros apps abertos: o S9 reduz o clock quando esquenta.

## 5. Ferramentas do Android Studio que você vai usar

- **Logcat** (aba embaixo): o log do celular. Os `print` do Python saem com as tags
  `python.stdout` e `python.stderr`; erros do app saem em vermelho (`AndroidRuntime`).
  Filtre digitando `python` na barra de busca.
- **Build** (aba embaixo): mensagens do Gradle e do `pip`. Se o build falhar, a causa fica aqui.
- **Device Explorer** (*View → Tool Windows*; antes "Device File Explorer"): navega nos
  arquivos do celular. Em apps de depuração, a pasta do app é `/data/data/com.example.carbench/`.
- Editar Python: o Android Studio mostra avisos como "No Python interpreter configured" e
  "Unresolved reference" nos `.py`. São inofensivos. **Edite o Python no seu editor do repositório**,
  não na pasta `app/src/main/python` (ela é regenerada).
- Mudou só Python? Basta clicar em **Run** de novo: o `syncPython` copia a versão nova.

## 6. Erros comuns

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| Build falha com "Python ... not found" / erro de `buildPython` | Não há Python **3.10** no PC | Instalar o 3.10 e conferir `py -3.10 --version`; ou fixar o caminho com `buildPython(...)` |
| `No matching distribution found for opencv-python-headless==4.5.1.48` | Python do app ≠ 3.10, ou ABI sem `arm64-v8a` | Conferir `version = "3.10"` e `abiFilters` |
| Erro de caminho longo / arquivo não encontrado durante o build | Limite de 260 caracteres do Windows | Mover o repositório para um caminho curto |
| `Sync` reclama da versão do Android Gradle Plugin | AGP fora da faixa testada | Ver a nota da seção 2.2 |
| App abre e o teste 1) mostra `ERRO: ... No module named 'cv2'` | Pacotes não foram instalados | Ver a aba *Build*; rodar *Build → Clean Project* e Run de novo |
| Teste 2) diz "Nao abriu o video" | OpenCV do Android sem decodificador de vídeo | Usar `assets/frames` e marcar a caixa de frames |
| App fecha ao iniciar | Falta o atributo `android:name` da `PyApplication` | Conferir a seção 2.3 e o Logcat |
| `ModuleNotFoundError: config` (ou `vision`) | O `syncPython` não copiou os arquivos | Ver se existe `app/src/main/python/config.py` depois do Sync/Run |

## 7. O que este esqueleto ainda NÃO faz
Câmera ao vivo (CameraX), USB com o ESP32, serviço em primeiro plano e a tela de
"motivação/pensamento". A ideia é crescer a partir daqui: primeiro medir (este passo),
depois ligar a câmera, depois o USB.