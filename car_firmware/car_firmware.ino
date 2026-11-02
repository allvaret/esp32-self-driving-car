/*
 * car_firmware.ino - carrinho autonomo (ESP32 + L298N + HC-SR04 em servo)       v0.2
 *
 * DOIS MODOS (troque por `mode auto|phone` na serial ou DEFAULT_MODE abaixo):
 *   auto  : o ESP32 decide sozinho so com o sensor (mesma logica do controller.py, sem visao):
 *           CRUISE -> SCAN (zona intermediaria) -> AVOID (para, olha L/R, re, curva) com
 *           guarda de reentrada. Serve para a demonstracao SEM celular.
 *   phone : o celular manda {"t":"aim"}, {"t":"drive"}, {"t":"cfg"} (JSON por linha) e o ESP32
 *           devolve {"t":"range","deg":..,"cm":..}. O ESP32 so faz o REFLEXO (parada por
 *           distancia, watchdog, limite de re).
 *
 * SEGURANCA (valem nos dois modos):
 *   - motores so giram ARMADO: botao BOOT (GPIO0) ou `go` na serial, com contagem de 3 s;
 *   - anda para frente SO com leitura central fresca e >= hard_stop_cm (sem leitura = parado);
 *   - phone: sem comando `drive` por wd_ms -> para;  re continua no maximo max_rev_ms.
 * Fiacao, comandos e testes de bancada: esp32/README_FIRMWARE.md
 * Valores [DEMO]: calibrar (ver docs/PARAMETROS_A_CALIBRAR.md). Mantenha coerente com config.py.
 */
#include <Arduino.h>
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "net.h"

#define FW_VERSION "car_firmware 0.2"

// ======================= OPCOES DE COMPILACAO ===============================
#define MOTORS_ENABLED 1        // 0 = bancada SEM motores: nada e escrito no L298N
#define STEER_SERVO 1           // direcao por servo (carro "esterca") + 2 motores de tracao
#define STEER_DIFFERENTIAL 2    // sem servo de direcao: curva por diferenca de velocidade
#define STEER_MODE STEER_DIFFERENTIAL
#define MODE_AUTO 0
#define MODE_PHONE 1
#define DEFAULT_MODE MODE_AUTO
#define MOTOR_BRAKE 1           // 1 = freio ativo ao parar (IN1=IN2=HIGH); 0 = roda livre
#define MOTOR_A_INVERT 1        // motor A (esquerdo) gira ao contrario? ponha 1
#define MOTOR_B_INVERT 1        // motor B (direito)
#define SENSOR_SERVO_SIGN -1     // +graus = direita. Se o servo girar ao contrario, ponha -1
#define STEER_SERVO_SIGN 1
#define HEARTBEAT_MS 500        // linha {"t":"state"...} periodica (0 = desliga)

// ================================ PINOS =====================================
// Evitados de proposito: 0/2/12/15 (boot), 6-11 (flash), 1/3 (USB serial), 34-39 (so entrada).
#define PIN_TRIG 23
#define PIN_ECHO 22             // ATENCAO: ECHO do HC-SR04 e 5 V -> divisor 1k/2k ate este pino
#define PIN_SENSOR_SERVO 18
#define PIN_STEER_SERVO 19
#define PIN_ENA 25              // L298N: ENA (PWM) - retire o jumper do ENA
#define PIN_IN1 26
#define PIN_IN2 27
#define PIN_ENB 32              // L298N: ENB (PWM) - retire o jumper do ENB
#define PIN_IN3 33
#define PIN_IN4 21
#define PIN_BTN 0               // botao BOOT da placa (arma/desarma)
#define PIN_LED 2               // LED da placa

#define CH_SENSOR_SERVO 0       // canais LEDC (core 2.x): 0/1 = 50 Hz, 2/3 = PWM dos motores
#define CH_STEER_SERVO 1
#define CH_ENA 2
#define CH_ENB 3

#define SERVO_CENTER_US 1500
#define SERVO_US_PER_DEG 11.0f
#define SERVO_MAX_DEG 80.0f
#define CENTER_TOL_DEG 10.0f    // |servo| <= isso conta como "olhando para frente"
#define MEASURE_PERIOD_MS 65    // HC-SR04: ciclo >= 60 ms
#define ARM_DELAY_MS 3000
#define MOTOR_PWM_HZ 1000

// ============================== PARAMETROS ==================================
struct Params {
  // distancias (cm)
  float d_stop_cm = 50, d_clear_cm = 100, hard_stop_cm = 20, none_score_cm = 50, max_range_cm = 200;
  // velocidades normalizadas [0,1]
  float v_cruise = 0.5f, v_slow = 0.2f, v_rev = 0.3f, v_turn = 0.3f;
  // manobra (ms) e servo
  float scan_deg = 45, stop_ms = 300, reverse_ms = 800, turn_ms = 1000, look_timeout_ms = 1000;
  float clear_ms = 200, avoid_retries = 2, settle_ms = 150;
  // guarda de reentrada
  float guard_window_ms = 20000, guard_max = 2, backoff1_ms = 1000, backoff2_ms = 2000, backoff3_ms = 4000;
  // carro (calibrar com a bateria)
  float trim_left = 1.0f, trim_right = 1.0f, steer_trim_deg = 0, steer_max_deg = 30, diff_gain = 0.6f;
  float pwm_min = 75, pwm_max = 140, slew_per_s = 4.0f;   // TT 3-6 V com 3S: ~3 V e ~6 V no motor
  // reflexo / phone
  float wd_ms = 500, fresh_ms = 400, max_rev_ms = 1500;
  float sensor_trim_deg = 0;              // [DEMO] calibrar: corrige o zero do servo do sensor
};
static Params P;

struct KV { const char* key; float* ptr; };
static const KV PARAM_TABLE[] = {
  {"d_stop_cm", &P.d_stop_cm}, {"d_clear_cm", &P.d_clear_cm}, {"hard_stop_cm", &P.hard_stop_cm},
  {"none_score_cm", &P.none_score_cm}, {"max_range_cm", &P.max_range_cm},
  {"v_cruise", &P.v_cruise}, {"v_slow", &P.v_slow}, {"v_rev", &P.v_rev}, {"v_turn", &P.v_turn},
  {"scan_deg", &P.scan_deg}, {"stop_ms", &P.stop_ms}, {"reverse_ms", &P.reverse_ms},
  {"turn_ms", &P.turn_ms}, {"look_timeout_ms", &P.look_timeout_ms}, {"clear_ms", &P.clear_ms},
  {"avoid_retries", &P.avoid_retries}, {"settle_ms", &P.settle_ms},
  {"guard_window_ms", &P.guard_window_ms}, {"guard_max", &P.guard_max},
  {"backoff1_ms", &P.backoff1_ms}, {"backoff2_ms", &P.backoff2_ms}, {"backoff3_ms", &P.backoff3_ms},
  {"trim_left", &P.trim_left}, {"trim_right", &P.trim_right}, {"steer_trim_deg", &P.steer_trim_deg},
  {"steer_max_deg", &P.steer_max_deg}, {"diff_gain", &P.diff_gain},
  {"pwm_min", &P.pwm_min}, {"pwm_max", &P.pwm_max}, {"slew_per_s", &P.slew_per_s},
  {"wd_ms", &P.wd_ms}, {"fresh_ms", &P.fresh_ms}, {"max_rev_ms", &P.max_rev_ms},
  {"sensor_trim_deg", &P.sensor_trim_deg},
};
static const int N_PARAMS = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

// ================================ ESTADO ====================================
static int mode = DEFAULT_MODE;
static bool armed = false, armPending = false;
static uint32_t armAt = 0;

static float servoDeg = 0;                 // angulo atual do servo do sensor
static bool servoInit = false;
static uint32_t tAim = 0;

struct Meas { bool valid; float cm; float raw; uint32_t t; float angle; };
static Meas meas = {false, -1, -1, 0, 0};
static float ringBuf[3];
static int ringN = 0;
static uint32_t tLastPing = 0;

static bool haveC = false;                 // ja houve leitura central assentada?
static float centerCm = -1, centerRaw = -1;  // -1 = sem eco (= livre)
static uint32_t centerT = 0;

static float cmdV = 0, cmdS = 0;           // ultimo drive do celular
static uint32_t lastDriveT = 0;

static float curL = 0, curR = 0;           // saida dos motores (apos rampa)
static uint32_t tLastApply = 0;
static bool revActive = false;
static uint32_t tRevStart = 0;
static float lastSteerOut = 999;

// maquina de estados (auto)
enum { C_CRUISE = 0, C_SCAN = 1, C_AVOID = 2 };
enum { S_NONE = 0, S_WAIT, S_STOP, S_LOOK_L, S_LOOK_R, S_REVERSE, S_TURN, S_HOLD };
static int cst = C_CRUISE, sub = S_NONE;
static uint32_t tSub = 0, waitUntil = 0, clearSince = 0;
static bool clearActive = false;
static float scoreL = 0, scoreR = 0, revMs = 800;
static int side = 0, tries = 0, guardLevel = 0;
static uint32_t avoidLog[8];
static int avoidN = 0;
static char why[56] = "";

static char lineBuf[160];
static int lineLen = 0;
static uint32_t tHeartbeat = 0;

// ============================== UTILIDADES ==================================
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float absf(float x) { return x < 0 ? -x : x; }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwmAttach(int pin, int ch, int hz, int bits) { (void)ch; ledcAttach(pin, hz, bits); }
static void pwmWrite(int pin, int ch, uint32_t duty) { (void)ch; ledcWrite(pin, duty); }
#else
static void pwmAttach(int pin, int ch, int hz, int bits) { ledcSetup(ch, hz, bits); ledcAttachPin(pin, ch); }
static void pwmWrite(int pin, int ch, uint32_t duty) { (void)pin; ledcWrite(ch, duty); }
#endif

static void servoUs(int pin, int ch, float us) {
  us = clampf(us, 500.0f, 2500.0f);
  pwmWrite(pin, ch, (uint32_t)(us * 65535.0f / 20000.0f));
}

static void setWhy(const char* s) { strncpy(why, s, sizeof(why) - 1); why[sizeof(why) - 1] = 0; }

// ------------------------- mini parser JSON (mensagens planas) --------------
static const char* jsonFind(const char* s, const char* key) {
  char pat[40];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(s, pat);
  if (!p) return NULL;
  p = strchr(p + strlen(pat), ':');
  if (!p) return NULL;
  p++;
  while (*p == ' ') p++;
  return p;
}
static bool jsonStr(const char* s, const char* key, char* out, size_t n) {
  const char* p = jsonFind(s, key);
  if (!p || *p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < n) out[i++] = *p++;
  out[i] = 0;
  return true;
}
static bool jsonNum(const char* s, const char* key, float* out) {
  const char* p = jsonFind(s, key);
  if (!p || !(*p == '-' || *p == '+' || *p == '.' || (*p >= '0' && *p <= '9'))) return false;
  *out = (float)atof(p);
  return true;
}
static bool jsonBool(const char* s, const char* key, bool* out) {
  const char* p = jsonFind(s, key);
  if (!p) return false;
  if (strncmp(p, "true", 4) == 0) { *out = true; return true; }
  if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
  return false;
}

// ================================ SERVO =====================================
static void aimTo(float deg, uint32_t now) {
  deg = clampf(deg, -SERVO_MAX_DEG, SERVO_MAX_DEG);
  if (servoInit && absf(deg - servoDeg) < 0.01f) return;
  servoInit = true;
  servoDeg = deg;
  servoUs(PIN_SENSOR_SERVO, CH_SENSOR_SERVO,
          SERVO_CENTER_US + SENSOR_SERVO_SIGN * (deg + P.sensor_trim_deg) * SERVO_US_PER_DEG);
  tAim = now;
  ringN = 0;
  meas.valid = false;
}

// ================================ SENSOR ====================================
static float pingRawCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, (unsigned long)(P.max_range_cm * 58.3f));
  if (us == 0) return -1.0f;                       // sem eco
  float cm = us / 58.3f;                           // cm = us * 0.0343 / 2
  return cm > P.max_range_cm ? -1.0f : cm;
}

static float medianKeep() {   // n=1: valor; n=2: o mais perto (conservador); n=3: mediana. -1 = sem eco
  float v[3];
  for (int i = 0; i < ringN; i++) v[i] = ringBuf[i] < 0 ? 1e9f : ringBuf[i];
  for (int i = 0; i < ringN; i++)
    for (int j = i + 1; j < ringN; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  float m = ringN == 1 ? v[0] : (ringN == 2 ? v[0] : v[1]);
  return m >= 1e8f ? -1.0f : m;
}

static void sendRange(float deg, float cm) {
  if (cm < 0) emitf("{\"t\":\"range\",\"deg\":%.1f,\"cm\":null}\n", deg);
  else emitf("{\"t\":\"range\",\"deg\":%.1f,\"cm\":%.1f}\n", deg, cm);
}

static void sensorTask(uint32_t now) {
  if (now - tLastPing < MEASURE_PERIOD_MS) return;
  tLastPing = now;
  float raw = pingRawCm();
  if (now - tAim < (uint32_t)P.settle_ms) return;  // servo ainda movendo: descarta
  if (ringN < 3) ringBuf[ringN++] = raw;
  else { ringBuf[0] = ringBuf[1]; ringBuf[1] = ringBuf[2]; ringBuf[2] = raw; }
  float med = medianKeep();
  uint32_t t = now;                                  // mesmo relogio do loop (evita diferenca negativa sem sinal)
  meas.valid = true; meas.cm = med; meas.raw = raw; meas.t = t; meas.angle = servoDeg;
  if (absf(servoDeg) <= CENTER_TOL_DEG) { centerCm = med; centerRaw = raw; centerT = t; haveC = true; }
  if (mode == MODE_PHONE || (netUp && ws.connectedClients() > 0)) sendRange(servoDeg, med);
}

static bool forwardAllowed(uint32_t now) {
  if (!haveC || (int32_t)(now - centerT) > (int32_t)P.fresh_ms) return false;  // sem leitura fresca: nao anda
  return centerRaw < 0 || centerRaw >= P.hard_stop_cm;                  // sem eco = livre
}

// ================================ MOTORES ===================================
static void motorWrite(int in1, int in2, int pin, int ch, float v) {
#if MOTORS_ENABLED
  float a = absf(v);
  if (a < 0.02f) {
#if MOTOR_BRAKE
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); pwmWrite(pin, ch, 255);
#else
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); pwmWrite(pin, ch, 0);
#endif
    return;
  }
  bool fwd = v > 0;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  float hi = clampf(P.pwm_max, P.pwm_min, 255.0f);         // protege contra pwm_max < pwm_min via `set`
  float duty = P.pwm_min + a * (hi - P.pwm_min);
  pwmWrite(pin, ch, (uint32_t)clampf(duty, 0.0f, hi));
#else
  (void)in1; (void)in2; (void)pin; (void)ch; (void)v;
#endif
}

static float slewTo(float cur, float target, float maxStep) {
  if (target * cur < 0) cur = 0;                         // inversao de sentido: passa por zero
  if (absf(target) <= absf(cur)) return target;          // desacelerar/parar e imediato
  float d = target - cur;
  return absf(d) <= maxStep ? target : cur + (d > 0 ? maxStep : -maxStep);
}

static void driveApply(uint32_t now, float v, float s) {
  if (!armed) { v = 0; s = 0; }
  if (v > 0 && !forwardAllowed(now)) v = 0;              // reflexo: sem leitura livre, nao avanca
  if (v < 0) {                                           // re e cega: limite de duracao continua
    if (!revActive) { revActive = true; tRevStart = now; }
    if (now - tRevStart > (uint32_t)P.max_rev_ms) v = 0;
  } else {
    revActive = false;
  }
  v = clampf(v, -1.0f, 1.0f);
  s = clampf(s, -1.0f, 1.0f);

  float L = v, R = v;
#if STEER_MODE == STEER_DIFFERENTIAL
  L = v + s * P.diff_gain;                               // s>0 = direita: esquerdo mais rapido
  R = v - s * P.diff_gain;
  float m = absf(L) > absf(R) ? absf(L) : absf(R);
  if (m > 1.0f) { L /= m; R /= m; }
#else
  float steerOut = STEER_SERVO_SIGN * (s * P.steer_max_deg + P.steer_trim_deg);
  if (absf(steerOut - lastSteerOut) > 0.05f) {
    servoUs(PIN_STEER_SERVO, CH_STEER_SERVO, SERVO_CENTER_US + steerOut * SERVO_US_PER_DEG);
    lastSteerOut = steerOut;
  }
#endif
  L = clampf(L * P.trim_left, -1.0f, 1.0f);
  R = clampf(R * P.trim_right, -1.0f, 1.0f);

  float dt = (float)(now - tLastApply) / 1000.0f;
  tLastApply = now;
  float step = P.slew_per_s * (dt > 0.2f ? 0.2f : dt);
  curL = slewTo(curL, L, step);
  curR = slewTo(curR, R, step);
  motorWrite(PIN_IN1, PIN_IN2, PIN_ENA, CH_ENA, MOTOR_A_INVERT ? -curL : curL);
  motorWrite(PIN_IN3, PIN_IN4, PIN_ENB, CH_ENB, MOTOR_B_INVERT ? -curR : curR);
}

// ============================ MAQUINA DE ESTADOS (auto) =====================
static void goSub(int s, uint32_t now) { sub = s; tSub = now; }

static void resetAuto(uint32_t now) {
  cst = C_CRUISE; sub = S_NONE; clearActive = false; avoidN = 0; guardLevel = 0; tries = 0;
  setWhy("");
  aimTo(0, now);
}

static void enterAvoid(uint32_t now, const char* reason) {
  int k = 0;
  for (int i = 0; i < avoidN; i++)
    if (now - avoidLog[i] <= (uint32_t)P.guard_window_ms) avoidLog[k++] = avoidLog[i];
  avoidN = k;
  if (avoidN == 0) guardLevel = 0;
  cst = C_AVOID; tries = 0;
  if (avoidN >= (int)P.guard_max) {                      // reentrada: espera com backoff
    float w = guardLevel == 0 ? P.backoff1_ms : (guardLevel == 1 ? P.backoff2_ms : P.backoff3_ms);
    guardLevel++;
    waitUntil = now + (uint32_t)w;
    snprintf(why, sizeof(why), "reentrada: espera %lu ms", (unsigned long)w);
    goSub(S_WAIT, now);
  } else {
    setWhy(reason);
    goSub(S_STOP, now);
  }
  if (avoidN >= 8) { for (int i = 1; i < 8; i++) avoidLog[i - 1] = avoidLog[i]; avoidN = 7; }
  avoidLog[avoidN++] = now;
}

static void decideSide(uint32_t now) {
  aimTo(0, now);
  if (scoreL < P.d_stop_cm && scoreR < P.d_stop_cm) {    // nenhum lado livre
    tries++;
    if (tries > (int)P.avoid_retries) { setWhy("sem saida"); clearActive = false; goSub(S_HOLD, now); return; }
    side = 0; revMs = P.reverse_ms * 1.5f;
  } else {
    side = scoreL >= scoreR ? -1 : +1; revMs = P.reverse_ms;
  }
  goSub(S_REVERSE, now);
}

static void autoStep(uint32_t now, float* v, float* s) {
  *v = 0; *s = 0;
  if (cst != C_AVOID) {                                  // ---- CRUISE / SCAN ----
    aimTo(0, now);
    if (!haveC) { setWhy("aguardando sensor"); return; }
    float d = centerCm;                                  // -1 = sem eco = livre
    if (d >= 0 && d < P.d_stop_cm) { enterAvoid(now, "sensor perto"); return; }
    bool mid = d >= 0 && d < P.d_clear_cm;
    if (mid) { cst = C_SCAN; clearActive = false; setWhy("zona intermediaria"); *v = P.v_slow; return; }
    if (cst == C_SCAN) {
      if (!clearActive) { clearActive = true; clearSince = now; }
      if (now - clearSince >= (uint32_t)P.clear_ms) { cst = C_CRUISE; clearActive = false; setWhy(""); *v = P.v_cruise; }
      else *v = P.v_slow;
      return;
    }
    *v = P.v_cruise;
    return;
  }
  uint32_t el = now - tSub;                              // ---- AVOID ----
  switch (sub) {
    case S_WAIT:
      if ((int32_t)(now - waitUntil) >= 0) { cst = C_SCAN; sub = S_NONE; clearActive = false; setWhy("fim da espera"); *v = P.v_slow; }
      return;
    case S_STOP:
      if (el >= (uint32_t)P.stop_ms) { aimTo(-P.scan_deg, now); goSub(S_LOOK_L, now); }
      return;
    case S_LOOK_L:
    case S_LOOK_R: {
      bool got = meas.valid && absf(meas.angle - servoDeg) < 0.5f;
      if (got || el >= (uint32_t)P.look_timeout_ms) {
        float sc = (got && meas.cm >= 0) ? meas.cm : P.none_score_cm;
        if (sub == S_LOOK_L) { scoreL = sc; aimTo(+P.scan_deg, now); goSub(S_LOOK_R, now); }
        else { scoreR = sc; decideSide(now); }
      }
      return;
    }
    case S_REVERSE:
      *v = -P.v_rev;
      if (el >= (uint32_t)revMs) {
        if (side == 0) { aimTo(-P.scan_deg, now); goSub(S_LOOK_L, now); }
        else goSub(S_TURN, now);
      }
      return;
    case S_TURN:
      *v = P.v_turn; *s = (float)side;
      if (el >= (uint32_t)P.turn_ms) { cst = C_SCAN; sub = S_NONE; clearActive = false; setWhy("pos-desvio"); *v = P.v_slow; *s = 0; }
      return;
    case S_HOLD: {                                       // preso: sai quando o centro libera
      aimTo(0, now);
      bool ok = centerCm < 0 || centerCm >= P.d_clear_cm;
      if (!ok) clearActive = false;
      else {
        if (!clearActive) { clearActive = true; clearSince = now; }
        if (now - clearSince >= (uint32_t)P.clear_ms) { cst = C_SCAN; sub = S_NONE; clearActive = false; setWhy("liberado"); }
      }
      return;
    }
    default: return;
  }
}

static void phoneStep(uint32_t now, float* v, float* s) {
  if (now - lastDriveT > (uint32_t)P.wd_ms) { *v = 0; *s = 0; return; }   // watchdog
  *v = cmdV; *s = cmdS;
}

// ============================ ARMAR / BOTAO / LED ===========================
static void setArmed(bool on, uint32_t now) {
  if (on) { armPending = true; armAt = now + ARM_DELAY_MS; emitf("{\"t\":\"log\",\"msg\":\"armando em 3 s\"}"); }
  else { armPending = false; armed = false; curL = curR = 0; emitf("{\"t\":\"log\",\"msg\":\"desarmado\"}"); }
}

static void buttonTask(uint32_t now) {
  static int last = HIGH;
  static uint32_t tChange = 0;
  int r = digitalRead(PIN_BTN);
  if (r != last && now - tChange > 60) {
    tChange = now;
    last = r;
    if (r == LOW) setArmed(!(armed || armPending), now);
  }
  if (armPending && (int32_t)(now - armAt) >= 0) {
    armPending = false; armed = true; resetAuto(now);
    lastDriveT = now;
    emitf("{\"t\":\"log\",\"msg\":\"ARMADO\"}");
  }
}

static void ledTask(uint32_t now) {
  bool on = armed ? true : (armPending ? ((now / 100) % 2) : ((now / 500) % 2));
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ================================ SERIAL ====================================
static void printHeartbeat(uint32_t now) {
  if (!HEARTBEAT_MS || now - tHeartbeat < HEARTBEAT_MS) return;
  tHeartbeat = now;
  static const char* CN[] = {"CRUISE", "SCAN", "AVOID"};
  static const char* SN[] = {"", "WAIT", "STOP", "LOOK_L", "LOOK_R", "REVERSE", "TURN", "HOLD"};
  emitf("{\"t\":\"state\",\"mode\":\"%s\",\"armed\":%s,\"st\":\"%s\",\"sub\":\"%s\",\"c\":%.1f,\"deg\":%.1f,"
                "\"L\":%.2f,\"R\":%.2f,\"why\":\"%s\"}\n",
                mode == MODE_AUTO ? "auto" : "phone", armed ? "true" : "false",
                mode == MODE_AUTO ? CN[cst] : "-", mode == MODE_AUTO ? SN[sub] : "",
                centerCm, servoDeg, curL, curR, why);
}

static void handleJson(const char* line, uint32_t now) {
  char t[16];
  if (!jsonStr(line, "t", t, sizeof(t))) return;
  float x;
  if (strcmp(t, "aim") == 0) {
    if (mode == MODE_PHONE && jsonNum(line, "deg", &x)) aimTo(x, now);
  } else if (strcmp(t, "drive") == 0) {
    float v = 0, s = 0;
    jsonNum(line, "v", &v); jsonNum(line, "s", &s);
    cmdV = v; cmdS = s; lastDriveT = now;
  } else if (strcmp(t, "cfg") == 0) {
    for (int i = 0; i < N_PARAMS; i++)
      if (jsonNum(line, PARAM_TABLE[i].key, &x)) *PARAM_TABLE[i].ptr = x;
    servoInit = false; aimTo(servoDeg, now);
    emitf("{\"t\":\"cfg_ok\"}");
  } else if (strcmp(t, "mode") == 0) {
    char m[12];
    if (jsonStr(line, "m", m, sizeof(m))) {
      mode = strcmp(m, "phone") == 0 ? MODE_PHONE : MODE_AUTO;
      cmdV = cmdS = 0; resetAuto(now);
    }
  } else if (strcmp(t, "arm") == 0) {
    bool on;
    if (jsonBool(line, "on", &on)) setArmed(on, now);
  } else if (strcmp(t, "ping") == 0) {
    emitf("{\"t\":\"pong\"}");
  } else if (strcmp(t, "goal") == 0) {          // motivacao vinda do visor
    char g[24];
    if (jsonStr(line, "g", g, sizeof(g))) {
      strncpy(netGoal, g, sizeof(netGoal) - 1);
      if (strcmp(g, "parar") == 0) setArmed(false, now);
      else if (mode == MODE_AUTO && !armed && !armPending) setArmed(true, now);
     }
     netRelay(line);
   } else if (strcmp(t, "think") == 0) {         // pensamento do cerebro -> visores
     netRelay(line);
    }
}

static void handleText(char* line, uint32_t now) {         // comandos digitaveis no Monitor Serial
  char a[24], b[24];
  float x = 0, y = 0;
  if (strcmp(line, "go") == 0) setArmed(true, now);
  else if (strcmp(line, "stop") == 0) setArmed(false, now);
  else if (sscanf(line, "mode %23s", a) == 1) {
    mode = strcmp(a, "phone") == 0 ? MODE_PHONE : MODE_AUTO;
    cmdV = cmdS = 0; resetAuto(now);
    Serial.printf("modo = %s\n", a);
  } else if (sscanf(line, "set %23s %f", a, &x) == 2) {
    bool ok = false;
    for (int i = 0; i < N_PARAMS; i++)
      if (strcmp(a, PARAM_TABLE[i].key) == 0) { *PARAM_TABLE[i].ptr = x; ok = true; }
    if (ok) { servoInit = false; aimTo(servoDeg, now); }   // reenvia o pulso com trim/params novos
    Serial.printf(ok ? "%s = %.3f\n" : "parametro desconhecido: %s (%.3f)\n", a, x);
  } else if (strcmp(line, "get") == 0) {
    for (int i = 0; i < N_PARAMS; i++) Serial.printf("%s = %.3f\n", PARAM_TABLE[i].key, *PARAM_TABLE[i].ptr);
  } else if (sscanf(line, "aim %f", &x) == 1) { aimTo(x, now); Serial.printf("aim %.1f\n", x); }
  else if (sscanf(line, "drive %f %f", &x, &y) == 2) { cmdV = x; cmdS = y; lastDriveT = now; Serial.printf("drive %.2f %.2f\n", x, y); }
  else if (strcmp(line, "status") == 0) { tHeartbeat = 0; printHeartbeat(now); }
  else if (strcmp(line, "help") == 0)
    Serial.println("go | stop | mode auto|phone | set <chave> <valor> | get | aim <graus> | drive <v> <s> | status");
  (void)b;
}

static void pollSerial(uint32_t now) {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        if (lineBuf[0] == '{') handleJson(lineBuf, now);
        else handleText(lineBuf, now);
      }
      lineLen = 0;
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = (char)c;
    } else {
      lineLen = 0;                                       // linha longa demais: descarta
    }
  }
}

// ================================= SETUP/LOOP ===============================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pwmAttach(PIN_SENSOR_SERVO, CH_SENSOR_SERVO, 50, 16);
#if STEER_MODE == STEER_SERVO
  pwmAttach(PIN_STEER_SERVO, CH_STEER_SERVO, 50, 16);
#endif
#if MOTORS_ENABLED
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pwmAttach(PIN_ENA, CH_ENA, MOTOR_PWM_HZ, 8);
  pwmAttach(PIN_ENB, CH_ENB, MOTOR_PWM_HZ, 8);
#endif
  uint32_t now = millis();
  aimTo(0, now);
  lastDriveT = now; tLastApply = now;
  driveApply(now, 0, 0);
  netBegin();
  emitf("{\"t\":\"hello\",\"fw\":\"%s\",\"mode\":\"%s\",\"armed\":false,\"motors\":%d}\n",
                FW_VERSION, mode == MODE_AUTO ? "auto" : "phone", MOTORS_ENABLED);
}

void loop() {
  uint32_t now = millis();
  pollSerial(now);
  netLoop();
  buttonTask(now);
  sensorTask(now);
  float v = 0, s = 0;
  if (armed) {
    if (mode == MODE_AUTO) autoStep(now, &v, &s);
    else phoneStep(now, &v, &s);
  }
  driveApply(now, v, s);
  printHeartbeat(now);
  ledTask(now);
}
