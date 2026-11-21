/*
 * car_firmware.ino - carrinho autonomo (ESP32 + L298N + HC-SR04 em servo)       v0.5.1
 *
 * CAMINHO 1: o ESP32 e o UNICO executor de manobras. O celular nao tem maquina de estados: so manda
 * a OPINIAO da visao ({"t":"vis"}) e o rumo do giroscopio ({"t":"yaw"}). Quem para, varre e desvia e este arquivo.
 *
 * DOIS MODOS (o visor escolhe pela FONTE do goal: "Explorar: sensor" = auto, "Explorar: visao" = phone):
 *   auto  : o ESP32 anda so com o sensor (nao usa a visao): FRENTE -> VARREDURA -> DESVIO (para, varre os
 *           angulos, escolhe o MELHOR, [re], curva ate la) com guarda de reentrada.
 *   phone : o celular manda {"t":"drive"} (v pela visao, s pelo rumo) e {"t":"avoid","deg":..} quando a visao
 *           ve obstaculo. O ESP32 faz os reflexos, VETA pelo sensor (mediana < d_stop_cm ou "sem eco suspeito")
 *           e executa o desvio sozinho; a visao so desempata os lados sem eco.
 *
 * "SEM ECO" TEM 3 ESTADOS (sempre pela MEDIANA e pelo historico, nunca pelo ping bruto):
 *   valida   : ha eco.
 *   livre    : sem eco E (visao fresca com setor 0 >= vis_free_min, ou nenhuma leitura < d_clear nos ultimos suspect_ms).
 *   suspeito : sem eco logo depois de leitura valida < d_clear (parede em diagonal / superficie macia),
 *              ou visao fresca com setor 0 < vis_free_min. Suspeito => para e varre (sem re inicial).
 *
 * SEGURANCA (valem nos dois modos):
 *   - motores so giram ARMADO: botao BOOT (GPIO0), `go` na serial, ou o goal "explorar" do visor, com contagem de 4 s;
 *   - anda para frente SO com leitura central fresca, classe nao-suspeita e mediana >= hard_stop_cm;
 *   - phone: sem comando `drive` por wd_ms -> ESP32 assume no modo sensor (fallback auto);
 *   - re continua no maximo max_rev_ms; partida escalonada e nunca com o servo se movendo.
 *   - RUMO EM RETA corrigido aqui (holdStep), nos dois modos, com o yaw do celular e um trim aprendido (NVS).
 *   - goal "parar", arm false ou BOOT desarmando: reset completo (modo padrao, FSM, RTC limpo).
 * Fiacao, comandos e testes de bancada: docs/protocolo_ws.md
 * Valores [DEMO]: calibrar (ver docs/parametros_a_calibrar.md). Mantenha coerente com config.py.
 */
#include <Arduino.h>
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>
#include <Preferences.h>        // NVS: guarda o trim aprendido do rumo entre desligamentos
#include "net.h"

#define FW_VERSION "car_firmware 0.5.1"

// ======================= OPCOES DE COMPILACAO ===============================
#define MOTORS_ENABLED 1        // 0 = bancada SEM motores: nada e escrito no L298N
#define STEER_SERVO 1           // direcao por servo (carro "esterca") + 2 motores de tracao
#define STEER_DIFFERENTIAL 2    // sem servo de direcao: curva por diferenca de velocidade
#define STEER_MODE STEER_DIFFERENTIAL
#define MODE_AUTO 0
#define MODE_PHONE 1
#define DEFAULT_MODE MODE_AUTO
#define MOTOR_BRAKE 0           // 1 = freio ativo ao parar (IN1=IN2=HIGH); 0 = roda livre (sem pico de frenagem)
// Fiacao real (MEDIDA pelo giroscopio nos logs de 20/11 e 21/11): ENA/IN1/IN2 (OUT1/OUT2) = roda dianteira ESQUERDA;
// ENB/IN3/IN4 (OUT3/OUT4) = roda dianteira DIREITA. A 0.5.0 tinha isso trocado (curvas ao contrario).
// So as dianteiras tem tracao (traseiras livres). L/curL/trim_left = roda ESQUERDA; R/curR/trim_right = roda DIREITA.
#define MOTOR_LEFT_INVERT 1     // roda ESQUERDA (ENA/IN1/IN2) gira ao contrario? ponha 1 (1 = sentido ja acertado no carro)
#define MOTOR_RIGHT_INVERT 1    // roda DIREITA (ENB/IN3/IN4)
#define SENSOR_SERVO_SIGN -1     // +graus = direita. Se o servo girar ao contrario, ponha -1
#define STEER_SERVO_SIGN 1
#define HEARTBEAT_MS 500        // linha {"t":"state"...} periodica (0 = desliga)

// ================================ PINOS =====================================
// Evitados de proposito: 0/2/12/15 (boot), 6-11 (flash), 1/3 (USB serial), 34-39 (so entrada).
#define PIN_TRIG 23
#define PIN_ECHO 22             // ATENCAO: ECHO do HC-SR04 e 5 V -> divisor 1k/2k ate este pino
#define PIN_SENSOR_SERVO 18
#define PIN_STEER_SERVO 4       // so no STEER_SERVO (nao usado no diferencial)
#define PIN_ENA 19              // L298N: ENA (PWM) -> OUT1/OUT2 -> roda ESQUERDA. 25/26/27 com defeito nesta placa
#define PIN_IN1 16              // RX2  (roda ESQUERDA)
#define PIN_IN2 17              // TX2  (roda ESQUERDA)
#define PIN_ENB 32              // L298N: ENB (PWM) -> OUT3/OUT4 -> roda DIREITA - retire o jumper do ENB
#define PIN_IN3 33              //      (roda DIREITA)
#define PIN_IN4 21              //      (roda DIREITA)
#define PIN_BTN 0               // botao BOOT da placa (arma/desarma)
#define PIN_LED 2               // LED da placa

#define CH_SENSOR_SERVO 0       // canais LEDC (core 2.x): 0/1 = 50 Hz, 2/3 = PWM dos motores
#define CH_STEER_SERVO 1
#define CH_ENA 2
#define CH_ENB 3

#define SERVO_CENTER_US 1500
#define SERVO_US_PER_DEG 11.0f
#define SERVO_MAX_DEG 80.0f
#define SERVO_STEP_DEG 15.0f    // servo do sensor anda em passos (= abertura do feixe do HC-SR04): evita o pico de corrente do salto
#define SERVO_STEP_MS 40        // intervalo entre passos
#define CENTER_TOL_DEG 10.0f    // |servo| <= isso conta como "olhando para frente"
#define MEASURE_PERIOD_MS 65    // HC-SR04: ciclo >= 60 ms
#define ARM_DELAY_MS 4000       // cobre a calibracao do chao no celular (o ESP32 ignora vis com ok:0)
#define MOTOR_PWM_HZ 1000
#define KICK_STAGGER_MS 60      // escalonamento: um motor parte 60 ms depois do outro
#define MAX_POS 6               // posicoes de varredura (3 passos de cada lado)

// ============================== PARAMETROS ==================================
struct Params {
  // distancias (cm)
  float d_stop_cm = 30, d_clear_cm = 60, hard_stop_cm = 20, none_score_cm = 45, max_range_cm = 200;
  // velocidades normalizadas [0,1]
  float v_cruise = 0.35f, v_slow = 0.15f, v_rev = 0.3f, v_turn = 0.3f;
  // manobra (ms) e servo. scan_deg/scan_step = posicoes da varredura (+-step, +-2*step ... ate scan_deg).
  // turn_dps = graus/s do carro com v_turn e s=1 (CALIBRAR): so vale sem yaw do celular.
  float scan_deg = 30, scan_step = 15, stop_ms = 300, reverse_ms = 500, turn_dps = 90, look_timeout_ms = 1000;
  float clear_ms = 200, avoid_retries = 2, settle_ms = 150;
  // curva com yaw: faltando menos de turn_slow_deg, o giro cai proporcional ao erro (fator >= turn_k_min, para nao parar)
  float turn_slow_deg = 25, turn_k_min = 0.35f;
  // guarda de reentrada
  float guard_window_ms = 20000, guard_max = 2, backoff1_ms = 1000, backoff2_ms = 2000, backoff3_ms = 4000;
  // carro (calibrar com a bateria). trim_left/right: ganho fixo da roda ESQUERDA/DIREITA fisica.
  // diff_gain: curva diferencial, L = v + s*diff_gain (roda esquerda), R = v - s*diff_gain (direita); s > 0 = direita.
  float trim_left = 1.0f, trim_right = 1.0f, steer_trim_deg = 0, steer_max_deg = 30, diff_gain = 0.6f;
  float pwm_min = 130, pwm_max = 150, slew_per_s = 2.0f;   // [DEMO] valores atuais do carro
  // faixa de duty POR RODA (comando 0..1 -> pwm_min_x..pwm_max_x); -1 = usa pwm_min/pwm_max. Medir com o teste por
  // roda (o visor sugere e aplica: mesma velocidade nas duas rodas para o mesmo comando). Gravados na NVS pelo cfg.
  float pwm_min_l = -1, pwm_max_l = -1, pwm_min_r = -1, pwm_max_r = -1;
  float kick_pwm = 200, kick_ms = 0;                         // arrancada desligada
  // reflexo / phone
  float wd_ms = 500, fresh_ms = 400, max_rev_ms = 1500;
  // visao e rumo vindos do celular (so phone usa vis). vis_free_min: setor 0 (0..100) a partir do qual a visao diz "livre".
  float vis_fresh_ms = 300, yaw_fresh_ms = 200, vis_free_min = 50, suspect_ms = 1500, yaw_lead_ms = 100;
  // RUMO EM RETA (correcao no ESP32, modos auto e phone). Trecho reto = |s| < hold_s_eps e v > 0.
  //   s_corr = -(hold_kp * giro + hold_ki * psi_trecho) + trim aprendido, limitado a +-hold_s_max
  // em UNIDADES DE RODA (0,1 = a roda lenta recebe +0,1 de comando; com pwm 130..150, 2 pontos de duty).
  // s_corr > 0 = mais forca na roda ESQUERDA (corrige um carro que puxa para a esquerda).
  // A roda lenta SOBE primeiro (ate hold_pwm_max de duty); a rapida so desce se a lenta saturar.
  float hold_kp = 0.01f, hold_ki = 0.05f, hold_s_max = 2.0f, hold_s_eps = 0.05f, hold_delay_ms = 100;
  float hold_psi_max = 30, hold_pwm_max = 200;
  // trim aprendido: integrador lento do termo hold_ki*psi (por s), persiste entre trechos e em NVS.
  float trim_rate = 0.5f, trim_max = 1.5f;
  float sensor_trim_deg = 0;              // [DEMO] corrige o zero do servo do sensor
  float fallback_auto = 1;                // 1 = sem celular, ESP32 segue so com o sensor; 0 = para
};
static Params P;

struct KV { const char* key; float* ptr; };
static const KV PARAM_TABLE[] = {
  {"d_stop_cm", &P.d_stop_cm}, {"d_clear_cm", &P.d_clear_cm}, {"hard_stop_cm", &P.hard_stop_cm},
  {"none_score_cm", &P.none_score_cm}, {"max_range_cm", &P.max_range_cm},
  {"v_cruise", &P.v_cruise}, {"v_slow", &P.v_slow}, {"v_rev", &P.v_rev}, {"v_turn", &P.v_turn},
  {"scan_deg", &P.scan_deg}, {"scan_step", &P.scan_step}, {"stop_ms", &P.stop_ms}, {"reverse_ms", &P.reverse_ms},
  {"turn_dps", &P.turn_dps}, {"look_timeout_ms", &P.look_timeout_ms}, {"clear_ms", &P.clear_ms},
  {"avoid_retries", &P.avoid_retries}, {"settle_ms", &P.settle_ms},
  {"turn_slow_deg", &P.turn_slow_deg}, {"turn_k_min", &P.turn_k_min},
  {"guard_window_ms", &P.guard_window_ms}, {"guard_max", &P.guard_max},
  {"backoff1_ms", &P.backoff1_ms}, {"backoff2_ms", &P.backoff2_ms}, {"backoff3_ms", &P.backoff3_ms},
  {"trim_left", &P.trim_left}, {"trim_right", &P.trim_right}, {"steer_trim_deg", &P.steer_trim_deg},
  {"steer_max_deg", &P.steer_max_deg}, {"diff_gain", &P.diff_gain},
  {"pwm_min", &P.pwm_min}, {"pwm_max", &P.pwm_max}, {"slew_per_s", &P.slew_per_s},
  {"pwm_min_l", &P.pwm_min_l}, {"pwm_max_l", &P.pwm_max_l}, {"pwm_min_r", &P.pwm_min_r}, {"pwm_max_r", &P.pwm_max_r},
  {"kick_pwm", &P.kick_pwm}, {"kick_ms", &P.kick_ms},
  {"wd_ms", &P.wd_ms}, {"fresh_ms", &P.fresh_ms}, {"max_rev_ms", &P.max_rev_ms},
  {"vis_fresh_ms", &P.vis_fresh_ms}, {"yaw_fresh_ms", &P.yaw_fresh_ms}, {"vis_free_min", &P.vis_free_min},
  {"suspect_ms", &P.suspect_ms}, {"yaw_lead_ms", &P.yaw_lead_ms},
  {"sensor_trim_deg", &P.sensor_trim_deg}, {"fallback_auto", &P.fallback_auto},
  {"hold_kp", &P.hold_kp}, {"hold_ki", &P.hold_ki}, {"hold_s_max", &P.hold_s_max}, {"hold_s_eps", &P.hold_s_eps},
  {"hold_delay_ms", &P.hold_delay_ms}, {"hold_psi_max", &P.hold_psi_max}, {"hold_pwm_max", &P.hold_pwm_max},
  {"trim_rate", &P.trim_rate}, {"trim_max", &P.trim_max},
};
static const int N_PARAMS = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

// ================================ ESTADO ====================================
static int mode = DEFAULT_MODE;
static bool armed = false, armPending = false;
static uint32_t armAt = 0;

static float servoDeg = 0;                 // angulo ALVO do servo do sensor (o que a FSM enxerga)
static float servoCur = 0;                 // angulo realmente comandado (anda ate o alvo em passos)
static bool servoInit = false;
static uint32_t tAim = 0, tServoStep = 0;

struct Meas { bool valid; float cm; float raw; uint32_t t; float angle; };
static Meas meas = {false, -1, -1, 0, 0};
static float ringBuf[3];
static int ringN = 0;
static uint32_t tLastPing = 0;

static bool haveC = false;                 // ja houve leitura central assentada?
static float centerCm = -1;                // MEDIANA central; -1 = sem eco
static uint32_t centerT = 0;
static bool haveNear = false;              // houve leitura central valida < d_clear?
static uint32_t nearT = 0;                 // quando (historico para classificar o "sem eco")

// opiniao da visao e rumo do celular
static bool visOk = false;
static int visS[5] = {0, 0, 0, 0, 0};      // setores livres 0..100 em -30,-15,0,+15,+30 graus
static uint32_t visT = 0;
static bool yawHave = false;
static float yawPsi = 0, yawR = 0;         // rumo acumulado (graus, + = direita) e giro (graus/s)
static uint32_t yawT = 0;

static float cmdV = 0, cmdS = 0;           // ultimo drive do celular
static uint32_t lastDriveT = 0;
static bool delegated = false;             // phone: ESP32 no comando (sem celular, veto do sensor ou desvio pedido)
static bool avoidReq = false;              // phone: celular pediu desvio
static bool avoidHintOk = false;           // ... com angulo sugerido
static float avoidHintDeg = 0;
static bool avoidFromPhone = false;        // o DESVIO atual veio de um avoid do celular (visao)
static bool avoidAgain = false;            // ... e o celular pediu de novo enquanto o ESP32 manobrava/esperava
static uint32_t tRefuteLog = 0;

static float curL = 0, curR = 0;           // saida da roda ESQUERDA / DIREITA (rampa + correcao de rumo)
static float rampL = 0, rampR = 0;         // comando base (v, s) depois da rampa, sem a correcao de rumo
static uint32_t tLastApply = 0;
static bool revActive = false;
static uint32_t tRevStart = 0;
static float lastSteerOut = 999;

// rumo em reta (holdStep)
enum { H_OFF = 0, H_TRIM = 1, H_PI = 2 };   // fora de reta | so trim (sem yaw fresco ou arrancando) | malha fechada
static int holdMode = H_OFF;
static bool holdOn = false, holdHavePsi0 = false, holdNoCorr = false;   // holdNoCorr: diagnostico "reto SEM correcao"
static uint32_t holdT0 = 0, holdTLearn = 0;
static float holdPsi0 = 0, holdCorr = 0;
static float holdTrim = 0, holdTrimSaved = 0;  // trim aprendido (unidades de roda) e o ultimo gravado em NVS
static Preferences prefs;

// diagnostico pelo visor ({"t":"diag","teste":..}): so armado; o FSM fica parado durante e DEPOIS do teste
enum { D_NONE = 0, D_DIR, D_ESQ, D_RETO, D_RETO_CORR, D_RODA_E, D_RODA_D };
static const char* DIAG_N[] = {"", "dir", "esq", "reto", "reto_corr", "roda_esq", "roda_dir"};
// teste POR RODA: so uma roda, para frente, com DUTY BRUTO em degraus (d0, d0+dd, ... ate d1; step_ms cada); a outra
// parada. O carro gira sobre a roda parada: o giro de cada degrau mede a velocidade daquela roda naquele duty.
#define WSTEP_MAX 16
static float wD0 = 110, wD1 = 230, wDD = 15, wStepMs = 300;
static int wDuty = 0;                        // > 0: driveApply escreve SO a roda do teste com este duty
static int wStep = -1, wN = 0, wBufN = 0;
static float wRates[WSTEP_MAX], wBuf[16];
static int wDuties[WSTEP_MAX];
static int diagTest = D_NONE;
static bool diagHold = false;              // armado mas parado (fim de um teste ou "armar" do diagnostico)
static uint32_t diagT0 = 0, diagDur = 0, diagYawT = 0;
static float diagPsi0 = 0, diagPsi1 = 0, diagSumL = 0, diagSumR = 0;
static bool diagHavePsi = false;
static int diagNLR = 0, diagN = 0;
#define DIAG_MAX 64
static float diagRates[DIAG_MAX];

// maquina de estados
enum { C_FRENTE = 0, C_VARREDURA = 1, C_DESVIO = 2 };
enum { S_NONE = 0, S_ESPERA, S_PARADA, S_OLHA, S_RE, S_CURVA, S_PRESO };
enum { R_VALID = 0, R_FREE = 1, R_SUSPECT = 2 };     // classe do centro
static int cst = C_FRENTE, sub = S_NONE;
static uint32_t tSub = 0, waitUntil = 0, clearSince = 0;
static bool clearActive = false;
static float revMs = 500;
static int tries = 0, guardLevel = 0;
static uint32_t avoidLog[8];
static int avoidN = 0;
static bool gone = true;                   // o caminho ja ficou livre desde o ultimo desvio?
static bool waitedAfter = false;           // uma ESPERA ja terminou em manobra (nao espera de novo sem o obstaculo sumir)
static bool noRevFlag = false;             // entrou por "suspeito": sem re inicial
static bool hintOk = false, hintFirst = false;
static float hintDeg = 0;
static float scanDeg_[MAX_POS], scanSc[MAX_POS];
static int scanOrd[MAX_POS], scanNPos = 0, scanIdx = 0;
static float turnTarget = 0, psi0 = 0, chosenAng = 0;
static bool turnYaw = false, haveAng = false;
#define TURN_REPORT_MS 300                 // {"t":"turn"} sai 300 ms depois do fim da curva (conta o giro por inercia)
static bool turnRepPending = false;
static uint32_t turnRepT = 0, turnStartT = 0, turnMs = 0;
static float turnRepAlvo = 0;
static char why[56] = "";

static char lineBuf[160];
static int lineLen = 0;
static uint32_t tHeartbeat = 0;
struct Kick { int dir; uint32_t until; };
static Kick kickL = {0, 0}, kickR = {0, 0};   // roda esquerda (ENA) / direita (ENB)

// sobrevive a reset por brownout/panic/watchdog (nao a desligar a energia)
#define RTC_MAGIC 0xCA770006u
RTC_NOINIT_ATTR uint32_t rtcMagic;
RTC_NOINIT_ATTR uint32_t rtcArmed;
RTC_NOINIT_ATTR uint32_t rtcMode;
RTC_NOINIT_ATTR uint32_t rtcRestores;
RTC_NOINIT_ATTR char rtcGoal[24];

// diario de bordo do servo/varredura (aim, olha, decide): sobrevive a reset (nao a falta de energia)
// e e reenviado a TODO cliente novo (visor e celular), assim a varredura nao se perde se o link cair
#define EV_N 16
struct ScanEv { uint32_t ms, n; float a, b; uint16_t boot; uint8_t k, ok, np; int8_t dg[MAX_POS]; float sc[MAX_POS]; };
RTC_NOINIT_ATTR ScanEv rtcEv[EV_N];
RTC_NOINIT_ATTR uint32_t rtcEvSeq;
RTC_NOINIT_ATTR uint32_t rtcBoot;
static int resetReason = 0;
static const char* RR[] = {"?", "LIGOU", "externo", "software", "PANIC", "wdt_int", "WDT_TASK", "WDT", "sono",
                           "BROWNOUT", "sdio"};
#define MAX_RESTORES 3                      // no maximo 3 retomadas seguidas sem desligar a energia

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
static bool jsonInts5(const char* s, const char* key, int* out) {   // "key":[a,b,c,d,e]
  const char* p = jsonFind(s, key);
  if (!p || *p != '[') return false;
  p++;
  for (int i = 0; i < 5; i++) {
    while (*p == ' ') p++;
    if (!(*p == '-' || (*p >= '0' && *p <= '9'))) return false;
    char* e;
    out[i] = (int)strtol(p, &e, 10);
    p = e;
    while (*p == ' ') p++;
    if (i < 4) { if (*p != ',') return false; p++; }
  }
  return true;
}

// ================================ SERVO =====================================
// k: 0 = aim (a = alvo, graus) | 1 = olha (a = graus, b = cm ou -1 sem eco, ok = 1 leu / 0 estourou o tempo)
//    2 = decide (a = angulo escolhido, ok = 1 sem saida; np posicoes dg[] com notas sc[], -1 = nao varrida)
static void evFormat(char* b, size_t n, const ScanEv& e, bool replay) {
  const char* kn = e.k == 0 ? "aim" : (e.k == 1 ? "olha" : "decide");
  int m = snprintf(b, n, "{\"t\":\"scan\",\"k\":\"%s\",\"n\":%lu,\"boot\":%u,\"ms\":%lu",
                   kn, (unsigned long)e.n, (unsigned)e.boot, (unsigned long)e.ms);
  if (m < 0 || (size_t)m >= n) return;
  if (e.k == 0) m += snprintf(b + m, n - m, ",\"deg\":%.1f", e.a);
  else if (e.k == 1) {
    m += snprintf(b + m, n - m, ",\"deg\":%.1f,\"got\":%d", e.a, (int)e.ok);
    if (e.ok && e.b >= 0) m += snprintf(b + m, n - m, ",\"cm\":%.1f", e.b);
    else m += snprintf(b + m, n - m, ",\"cm\":null");
  } else {
    if (e.ok) m += snprintf(b + m, n - m, ",\"ang\":null,\"x\":1");
    else m += snprintf(b + m, n - m, ",\"ang\":%.0f,\"x\":0", e.a);
    m += snprintf(b + m, n - m, ",\"d\":[");
    for (int i = 0; i < e.np && (size_t)m < n; i++) m += snprintf(b + m, n - m, "%s%d", i ? "," : "", (int)e.dg[i]);
    m += snprintf(b + m, n - m, "],\"sc\":[");
    for (int i = 0; i < e.np && (size_t)m < n; i++) m += snprintf(b + m, n - m, "%s%.0f", i ? "," : "", e.sc[i]);
    if ((size_t)m < n) m += snprintf(b + m, n - m, "]");
  }
  if (replay && (size_t)m < n) m += snprintf(b + m, n - m, ",\"replay\":1");
  if ((size_t)m < n) snprintf(b + m, n - m, "}");
}

static ScanEv& evNew(uint8_t k, uint32_t now) {
  uint32_t n = ++rtcEvSeq;
  ScanEv& e = rtcEv[n % EV_N];
  memset(&e, 0, sizeof(e));
  e.ms = now; e.n = n; e.boot = (uint16_t)rtcBoot; e.k = k;
  return e;
}

static void evEmit(const ScanEv& e) {
  char buf[240];
  evFormat(buf, sizeof(buf), e, false);
  emitf("%s", buf);
}

static void logEv(uint8_t k, float a, float b, uint8_t ok, uint32_t now) {
  ScanEv& e = evNew(k, now);
  e.a = a; e.b = b; e.ok = ok;
  evEmit(e);
}

static void logDecide(float ang, bool noExit, uint32_t now) {     // angulo escolhido e as notas de cada posicao
  ScanEv& e = evNew(2, now);
  e.a = ang; e.ok = noExit ? 1 : 0; e.np = (uint8_t)scanNPos;
  for (int i = 0; i < scanNPos; i++) { e.dg[i] = (int8_t)lroundf(scanDeg_[i]); e.sc[i] = scanSc[i]; }
  evEmit(e);
}

static void onWsConnect(uint8_t num) {                   // todo cliente novo recebe o diario (reenviado: "replay":1)
  uint32_t first = rtcEvSeq > EV_N ? rtcEvSeq - EV_N + 1 : 1;
  char buf[240];
  for (uint32_t n = first; n <= rtcEvSeq; n++) {
    const ScanEv& e = rtcEv[n % EV_N];
    if (e.n != n) continue;
    evFormat(buf, sizeof(buf), e, true);
    ws.sendTXT(num, buf);
  }
}

static void servoWrite(float deg) {
  servoUs(PIN_SENSOR_SERVO, CH_SENSOR_SERVO,
          SERVO_CENTER_US + SENSOR_SERVO_SIGN * (deg + P.sensor_trim_deg) * SERVO_US_PER_DEG);
}

static void aimTo(float deg, uint32_t now) {
  deg = clampf(deg, -SERVO_MAX_DEG, SERVO_MAX_DEG);
  if (servoInit && absf(deg - servoDeg) < 0.01f) return;
  bool first = !servoInit;                               // 1a vez / reaplicar trim: vai direto, sem rampa
  servoInit = true;
  servoDeg = deg;
  if (first) { servoCur = deg; servoWrite(deg); }
  else { tServoStep = now - SERVO_STEP_MS; logEv(0, deg, 0, 0, now); }   // 1o passo ja no proximo servoTick
  tAim = now;
  ringN = 0;
  meas.valid = false;
}

static void servoTick(uint32_t now) {                    // anda ate o alvo em passos de SERVO_STEP_DEG
  float d = servoDeg - servoCur;
  if (absf(d) < 0.01f) return;
  if (now - tServoStep < (uint32_t)SERVO_STEP_MS) return;
  tServoStep = now;
  servoCur = absf(d) <= SERVO_STEP_DEG ? servoDeg : servoCur + (d > 0 ? SERVO_STEP_DEG : -SERVO_STEP_DEG);
  servoWrite(servoCur);
  tAim = now;                                            // o "assentamento" conta a partir do ultimo passo
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
  if (cm < 0) emitf("{\"t\":\"range\",\"deg\":%.1f,\"cm\":null}", deg);
  else emitf("{\"t\":\"range\",\"deg\":%.1f,\"cm\":%.1f}", deg, cm);
}

static void sensorTask(uint32_t now) {
  if (now - tLastPing < MEASURE_PERIOD_MS) return;
  tLastPing = now;
  float raw = pingRawCm();
  if (absf(servoDeg - servoCur) > 0.01f) return;   // servo ainda andando em passos: descarta
  if (now - tAim < (uint32_t)P.settle_ms) return;  // servo ainda movendo: descarta
  if (ringN < 3) ringBuf[ringN++] = raw;
  else { ringBuf[0] = ringBuf[1]; ringBuf[1] = ringBuf[2]; ringBuf[2] = raw; }
  float med = medianKeep();
  uint32_t t = now;                                  // mesmo relogio do loop (evita diferenca negativa sem sinal)
  meas.valid = true; meas.cm = med; meas.raw = raw; meas.t = t; meas.angle = servoDeg;
  if (absf(servoDeg) <= CENTER_TOL_DEG) {
    centerCm = med; centerT = t; haveC = true;
    if (med >= 0 && med < P.d_clear_cm) { haveNear = true; nearT = t; }   // historico para o "sem eco suspeito"
  }
  if (mode == MODE_PHONE || (netUp && ws.connectedClients() > 0)) sendRange(servoDeg, med);
}

// ---- opiniao do celular e classe do centro ----
static bool visUsable(uint32_t now) {                    // so o modo phone usa a visao; velha ou ok:0 = sem opiniao
  return mode == MODE_PHONE && visOk && (int32_t)(now - visT) <= (int32_t)P.vis_fresh_ms;
}
static bool yawFresh(uint32_t now) { return yawHave && (int32_t)(now - yawT) <= (int32_t)P.yaw_fresh_ms; }
static float yawPsiNow(uint32_t now) {                   // psi extrapolado pelo giro desde a ultima amostra (~50 ms)
  int32_t age = (int32_t)(now - yawT);
  if (age < 0) age = 0;
  if (age > (int32_t)P.yaw_fresh_ms) age = (int32_t)P.yaw_fresh_ms;
  return yawPsi + yawR * (float)age / 1000.0f;
}

static int sectorOf(float deg) {                         // setor da visao mais proximo do angulo (-30..+30 em passos de 15)
  int i = (int)lroundf(deg / 15.0f) + 2;
  return i < 0 ? 0 : (i > 4 ? 4 : i);
}

static int centerClass(uint32_t now) {                   // usa a MEDIANA (centerCm) e o historico; nunca o ping bruto
  if (centerCm >= 0) return R_VALID;
  if (visUsable(now)) return visS[2] >= (int)P.vis_free_min ? R_FREE : R_SUSPECT;
  if (haveNear && (int32_t)(now - nearT) <= (int32_t)P.suspect_ms) return R_SUSPECT;
  return R_FREE;
}

static float noEchoScore(float deg, uint32_t now) {      // nota de uma posicao SEM eco
  if (visUsable(now)) return visS[sectorOf(deg)] >= (int)P.vis_free_min ? P.max_range_cm : P.d_stop_cm * 0.5f;
  return P.none_score_cm;
}

// leitura central VALIDA, fresca e >= d_clear_cm: o sensor ve o caminho livre e REFUTA um "obstaculo" da visao
static bool sensorRefutes(uint32_t now) {
  return haveC && (int32_t)(now - centerT) <= (int32_t)P.fresh_ms && centerCm >= P.d_clear_cm;
}

static bool forwardAllowed(uint32_t now) {
  if (!haveC || (int32_t)(now - centerT) > (int32_t)P.fresh_ms) return false;  // sem leitura fresca: nao anda
  int c = centerClass(now);
  if (c == R_SUSPECT) return false;                      // sem eco suspeito: nao anda (a FSM varre)
  if (c == R_VALID) return centerCm >= P.hard_stop_cm;   // mediana, nao o ping bruto
  return true;                                           // livre
}

// ================================ MOTORES ===================================
// faixa de duty de cada roda (w: 0 = esquerda, 1 = direita): pwm_min_l/r e pwm_max_l/r; -1 = pwm_min/pwm_max
static float wheelLo(int w) { float x = w ? P.pwm_min_r : P.pwm_min_l; return clampf(x >= 0 ? x : P.pwm_min, 0.0f, 255.0f); }
static float wheelHi(int w) { float x = w ? P.pwm_max_r : P.pwm_max_l; return clampf(x >= 0 ? x : P.pwm_max, wheelLo(w), 255.0f); }

static void motorWrite(int w, int in1, int in2, int pin, int ch, float v, Kick& k, uint32_t now, uint32_t delayMs) {
#if MOTORS_ENABLED
  float a = absf(v);
  if (a < 0.02f) {
    k.dir = 0;                                             // parado: a proxima saida ganha pulso
#if MOTOR_BRAKE
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); pwmWrite(pin, ch, 255);
#else
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); pwmWrite(pin, ch, 0);
#endif
    return;
  }
  int dir = v > 0 ? 1 : -1;
  if (dir != k.dir) { k.dir = dir; k.until = now + delayMs + (uint32_t)P.kick_ms; }  // saiu do zero/inverteu
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW : HIGH);
  float lo = wheelLo(w), hi = wheelHi(w);
  float duty = lo + a * (hi - lo);
  int32_t left = (int32_t)(k.until - now);
  if (left > 0) {
    if (left > (int32_t)P.kick_ms) duty = 0;               // escalonamento: espera a vez de partir
    else if (P.kick_pwm > duty) duty = P.kick_pwm;         // arrancada
  }
  pwmWrite(pin, ch, (uint32_t)clampf(duty, 0.0f, 255.0f));
#else
  (void)in1; (void)in2; (void)pin; (void)ch; (void)v; (void)k; (void)now; (void)delayMs;
#endif
}

static float slewTo(float cur, float target, float maxStep) {
  if (target * cur < 0) cur = 0;                         // inversao de sentido: passa por zero
  if (absf(target) <= absf(cur)) return target;          // desacelerar/parar e imediato
  float d = target - cur;
  return absf(d) <= maxStep ? target : cur + (d > 0 ? maxStep : -maxStep);
}

// ---- rumo em reta: correcao no ESP32 (o celular manda s = 0 na reta; nao ha mais correcao no brain) ----
// Trecho reto = v > 0 e |s| < hold_s_eps (FRENTE, VARREDURA, drive reto do celular). Corrige desde a arrancada:
// o trim aprendido vale ja no 1o ciclo; o PI entra apos hold_delay_ms se o yaw estiver fresco (sem yaw: so o trim).
// psi_trecho = psi desde o inicio do trecho (anti-windup: +-hold_psi_max). O trim integra devagar o termo
// hold_ki*psi_trecho e NAO zera entre trechos (fica o desvio fixo das rodas). Retorna a correcao (unid. de roda).
static float holdStep(uint32_t now, float v, float s) {
  bool straight = v > 0.02f && absf(s) < P.hold_s_eps;
  if (!straight) { holdOn = false; holdMode = H_OFF; holdCorr = 0; return 0; }
  if (!holdOn) { holdOn = true; holdT0 = now; holdTLearn = now; holdHavePsi0 = false; }
  float dt = (float)(now - holdTLearn) / 1000.0f;
  holdTLearn = now;
  if (holdNoCorr) { holdMode = H_OFF; holdCorr = 0; return 0; }
  bool fresh = yawFresh(now);
  if (fresh && !holdHavePsi0) { holdHavePsi0 = true; holdPsi0 = yawPsi; }
  float corr = holdTrim;
  holdMode = H_TRIM;
  if (fresh && holdHavePsi0 && now - holdT0 >= (uint32_t)P.hold_delay_ms) {
    float psiT = yawPsi - holdPsi0;
    if (psiT > P.hold_psi_max) { holdPsi0 = yawPsi - P.hold_psi_max; psiT = P.hold_psi_max; }
    else if (psiT < -P.hold_psi_max) { holdPsi0 = yawPsi + P.hold_psi_max; psiT = -P.hold_psi_max; }
    float iTerm = -P.hold_ki * psiT;
    corr += -P.hold_kp * yawR + iTerm;
    if (dt > 0 && dt < 0.2f) holdTrim = clampf(holdTrim + P.trim_rate * iTerm * dt, -P.trim_max, P.trim_max);
    holdMode = H_PI;
  }
  holdCorr = clampf(corr, -P.hold_s_max, P.hold_s_max);
  return holdCorr;
}

static void saveTrim() {                                 // so ao desarmar (poucas gravacoes na flash)
  if (absf(holdTrim - holdTrimSaved) < 0.01f) return;
  prefs.begin("carro", false);
  prefs.putFloat("trim", holdTrim);
  prefs.end();
  holdTrimSaved = holdTrim;
  emitf("{\"t\":\"log\",\"msg\":\"trim do rumo salvo: %.3f\"}", holdTrim);
}

static void motorRaw(int in1, int in2, int pin, int ch, int dir, int duty, Kick& k) {   // duty bruto (teste por roda)
#if MOTORS_ENABLED
  k.dir = duty > 0 ? dir : 0;
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW : HIGH);
  pwmWrite(pin, ch, (uint32_t)(duty < 0 ? 0 : (duty > 255 ? 255 : duty)));
#else
  (void)in1; (void)in2; (void)pin; (void)ch; (void)dir; (void)duty; (void)k;
#endif
}

static void saveWheels() {                               // faixa de duty por roda -> NVS (so quando o cfg muda)
  prefs.begin("carro", false);
  prefs.putFloat("pmin_l", P.pwm_min_l); prefs.putFloat("pmax_l", P.pwm_max_l);
  prefs.putFloat("pmin_r", P.pwm_min_r); prefs.putFloat("pmax_r", P.pwm_max_r);
  prefs.end();
  emitf("{\"t\":\"log\",\"msg\":\"rodas (NVS): esq %.0f..%.0f dir %.0f..%.0f\"}", wheelLo(0), wheelHi(0), wheelLo(1), wheelHi(1));
}

static void driveApply(uint32_t now, float v, float s) {
  if (!armed) { v = 0; s = 0; }
#if STEER_MODE == STEER_DIFFERENTIAL
  if (armed && wDuty > 0 && (diagTest == D_RODA_E || diagTest == D_RODA_D)) {
    // teste por roda: SO a roda pedida, para frente, com duty bruto (sem rampa, trim nem correcao); a outra parada.
    // Reflexos: nao parte com o servo girando e nao avanca sem leitura livre. state L/R = duty/255.
    bool left = diagTest == D_RODA_E;
    bool block = ((now - tAim) < (uint32_t)P.settle_ms) || !forwardAllowed(now);
    int d = block ? 0 : wDuty;
    rampL = rampR = 0;
    curL = left ? d / 255.0f : 0; curR = left ? 0 : d / 255.0f;
    holdStep(now, 0, 0);                                   // fora de reta
    tLastApply = now;
    if (left) {
      motorRaw(PIN_IN1, PIN_IN2, PIN_ENA, CH_ENA, MOTOR_LEFT_INVERT ? -1 : 1, d, kickL);
      motorWrite(1, PIN_IN3, PIN_IN4, PIN_ENB, CH_ENB, 0, kickR, now, 0);
    } else {
      motorRaw(PIN_IN3, PIN_IN4, PIN_ENB, CH_ENB, MOTOR_RIGHT_INVERT ? -1 : 1, d, kickR);
      motorWrite(0, PIN_IN1, PIN_IN2, PIN_ENA, CH_ENA, 0, kickL, now, 0);
    }
    return;
  }
#endif
  bool servoMoving = (now - tAim) < (uint32_t)P.settle_ms;
  if (servoMoving && absf(curL) < 0.02f && absf(curR) < 0.02f) { v = 0; s = 0; }  // nao parte com servo girando
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
  L = v + s * P.diff_gain;                               // s>0 = direita: roda ESQUERDA mais rapida
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
  rampL = slewTo(rampL, L, step);                        // a RAMPA vale para o comando base (v, s)...
  rampR = slewTo(rampR, R, step);
  curL = rampL; curR = rampR;
#if STEER_MODE == STEER_DIFFERENTIAL
  // ... e a correcao de rumo entra POR CIMA, sem esperar a rampa: na arrancada a roda forte ja anda no pwm_min e a
  // fraca nao; se a correcao subisse na rampa (2/s) o carro giraria em toda partida.
  float hc = holdStep(now, v, s);                        // 0 fora de trecho reto
  if (hc != 0) {                                         // sobe a roda LENTA; a rapida so desce se a lenta saturar
    int ws = hc > 0 ? 0 : 1;                             // roda lenta: a faixa de duty DELA define o teto
    float span = wheelHi(ws) - wheelLo(ws);
    float lim = span > 1 ? clampf((P.hold_pwm_max - wheelLo(ws)) / span, 1.0f, 10.0f) : 1.0f;   // teto: hold_pwm_max
    float* slow = hc > 0 ? &curL : &curR;
    float* fast = hc > 0 ? &curR : &curL;
    *slow += absf(hc);
    if (*slow > lim) { *fast -= *slow - lim; *slow = lim; }
    if (*fast < 0.05f) *fast = 0.05f;                    // a correcao nunca para nem inverte uma roda
  }
#endif
  // escalonamento: parte primeiro a roda com MAIS comando (a lenta, quando a correcao atua); a outra espera
  // KICK_STAGGER_MS. Sem correcao (L = R) a ESQUERDA (ENA) parte primeiro.
  if (absf(curL) >= absf(curR)) {
    motorWrite(0, PIN_IN1, PIN_IN2, PIN_ENA, CH_ENA, MOTOR_LEFT_INVERT ? -curL : curL, kickL, now, 0);
    motorWrite(1, PIN_IN3, PIN_IN4, PIN_ENB, CH_ENB, MOTOR_RIGHT_INVERT ? -curR : curR, kickR, now,
               kickL.dir != 0 ? KICK_STAGGER_MS : 0);
  } else {
    motorWrite(1, PIN_IN3, PIN_IN4, PIN_ENB, CH_ENB, MOTOR_RIGHT_INVERT ? -curR : curR, kickR, now, 0);
    motorWrite(0, PIN_IN1, PIN_IN2, PIN_ENA, CH_ENA, MOTOR_LEFT_INVERT ? -curL : curL, kickL, now,
               kickR.dir != 0 ? KICK_STAGGER_MS : 0);
  }
}

// ============================ MAQUINA DE ESTADOS ============================
static void goSub(int s, uint32_t now) { sub = s; tSub = now; }

static void pruneAvoid(uint32_t now) {
  int k = 0;
  for (int i = 0; i < avoidN; i++)
    if (now - avoidLog[i] <= (uint32_t)P.guard_window_ms) avoidLog[k++] = avoidLog[i];
  avoidN = k;
}

static void guardDecay(uint32_t now) {                   // o historico da guarda envelhece com o tempo (nao so ao reentrar)
  pruneAvoid(now);
  if (avoidN == 0) guardLevel = 0;
}

static void resetAuto(uint32_t now) {
  cst = C_FRENTE; sub = S_NONE; clearActive = false; avoidN = 0; guardLevel = 0; tries = 0;
  gone = true; waitedAfter = false; noRevFlag = false; hintOk = false; hintFirst = false; haveAng = false;
  avoidFromPhone = false; avoidAgain = false;
  setWhy("");
  aimTo(0, now);
}

// entra em DESVIO. noRev: veio de "sem eco suspeito" (varre sem re inicial). force: ignora a guarda (fim da ESPERA).
// A guarda so manda ESPERAR se ja houve guard_max desvios na janela E (nao esperou ainda OU o obstaculo sumiu e voltou).
static void enterAvoid(uint32_t now, const char* reason, bool noRev, bool force) {
  pruneAvoid(now);
  if (avoidN == 0) guardLevel = 0;
  cst = C_DESVIO; tries = 0; noRevFlag = noRev; haveAng = false; avoidAgain = false;
  bool wait = !force && avoidN >= (int)P.guard_max && !(waitedAfter && !gone);
  gone = false;
  if (wait) {                                            // reentrada: espera com backoff
    float w = guardLevel == 0 ? P.backoff1_ms : (guardLevel == 1 ? P.backoff2_ms : P.backoff3_ms);
    guardLevel++;
    waitUntil = now + (uint32_t)w;
    snprintf(why, sizeof(why), "reentrada: espera %lu ms", (unsigned long)w);
    goSub(S_ESPERA, now);
  } else {
    setWhy(reason);
    goSub(S_PARADA, now);
  }
  if (avoidN >= 8) { for (int i = 1; i < 8; i++) avoidLog[i - 1] = avoidLog[i]; avoidN = 7; }
  avoidLog[avoidN++] = now;
}

// posicoes de varredura (crescente): +-step, +-2*step ... ate scan_deg. Ordem: vizinho mais proximo, comecando
// pelo angulo sugerido pelo celular (se houver), senao pelo centro.
static void buildScan(uint32_t now) {
  (void)now;
  int k = (int)(P.scan_deg / (P.scan_step > 1 ? P.scan_step : 1) + 0.5f);
  if (k < 1) k = 1;
  if (k > MAX_POS / 2) k = MAX_POS / 2;
  scanNPos = 0;
  for (int i = -k; i <= k; i++) if (i != 0) scanDeg_[scanNPos++] = i * (P.scan_step > 1 ? P.scan_step : 1);
  bool used[MAX_POS];
  for (int i = 0; i < scanNPos; i++) { used[i] = false; scanSc[i] = -1; }
  float cur = 0;
  hintFirst = hintOk;
  for (int n = 0; n < scanNPos; n++) {
    int best = -1;
    for (int i = 0; i < scanNPos; i++) {
      if (used[i]) continue;
      float ref = (n == 0 && hintOk) ? hintDeg : cur;
      if (best < 0 || absf(scanDeg_[i] - ref) < absf(scanDeg_[best] - ref)) best = i;   // empate: o da esquerda (crescente)
    }
    used[best] = true;
    scanOrd[n] = best;
    cur = scanDeg_[best];
  }
  scanIdx = 0;
  hintOk = false;
}

static void startTurn(uint32_t now) {
  turnYaw = yawFresh(now);                               // com yaw: gira ate o delta; sem: pelo tempo (turn_dps)
  psi0 = turnYaw ? yawPsiNow(now) : yawPsi;
  turnStartT = now;
  goSub(S_CURVA, now);
}

static void turnEnd(uint32_t now) {                      // fim da curva: o relatorio sai TURN_REPORT_MS depois
  turnRepPending = true; turnRepT = now; turnMs = now - turnStartT; turnRepAlvo = turnTarget;
}

static void turnReportTask(uint32_t now) {               // {"t":"turn","alvo","girou","erro"}: erro = girou - alvo
  if (!turnRepPending || now - turnRepT < TURN_REPORT_MS) return;
  turnRepPending = false;
  if (yawFresh(now)) {
    float g = yawPsi - psi0;
    emitf("{\"t\":\"turn\",\"alvo\":%.0f,\"girou\":%.1f,\"erro\":%.1f,\"ms\":%lu,\"yaw\":%d}",
          turnRepAlvo, g, g - turnRepAlvo, (unsigned long)turnMs, turnYaw ? 1 : 0);
  } else {
    emitf("{\"t\":\"turn\",\"alvo\":%.0f,\"girou\":null,\"erro\":null,\"ms\":%lu,\"yaw\":0}", turnRepAlvo, (unsigned long)turnMs);
  }
}

// escolhe o MELHOR ANGULO. forced = angulo ja aceito (sugestao do celular com folga); NAN = escolher entre as notas.
// Empate: menor |angulo|, depois esquerda.
static void decideAngle(uint32_t now, float forced) {
  aimTo(0, now);
  bool have = false;
  float bestSc = -1, bestAng = 0;
  if (!isnan(forced)) {
    for (int i = 0; i < scanNPos; i++) if (scanDeg_[i] == forced) { bestSc = scanSc[i]; bestAng = forced; have = true; }
  } else {
    for (int i = 0; i < scanNPos; i++) {
      float sc = scanSc[i], a = scanDeg_[i];
      if (sc < 0) continue;
      if (!have || sc > bestSc || (sc == bestSc && (absf(a) < absf(bestAng) || (absf(a) == absf(bestAng) && a < bestAng)))) {
        have = true; bestSc = sc; bestAng = a;
      }
    }
  }
  if (!have || bestSc < P.d_stop_cm) {                   // nenhum angulo livre
    tries++;
    if (tries > (int)P.avoid_retries) { setWhy("sem saida"); clearActive = false; logDecide(0, true, now); goSub(S_PRESO, now); return; }
    turnTarget = 0; revMs = P.reverse_ms * 1.5f; noRevFlag = false; haveAng = false;
    logDecide(0, true, now);
    goSub(S_RE, now);
    return;
  }
  turnTarget = bestAng; chosenAng = bestAng; haveAng = true; revMs = P.reverse_ms;
  logDecide(bestAng, false, now);
  if (noRevFlag) startTurn(now); else goSub(S_RE, now);  // "suspeito": gira direto, sem re
}

static void autoStep(uint32_t now, float* v, float* s) {
  *v = 0; *s = 0;
  if (cst != C_DESVIO) {                                  // ---- FRENTE / VARREDURA ----
    aimTo(0, now);
    if (!haveC) { setWhy("aguardando sensor"); return; }
    guardDecay(now);
    int cls = centerClass(now);
    float d = centerCm;                                  // -1 = sem eco
    if (cls == R_VALID && d < P.d_stop_cm) { enterAvoid(now, "sensor perto", false, false); return; }
    if (cls == R_SUSPECT) { enterAvoid(now, "sem eco suspeito", true, false); return; }
    bool mid = cls == R_VALID && d < P.d_clear_cm;
    if (mid) { cst = C_VARREDURA; clearActive = false; setWhy("zona intermediaria"); *v = P.v_slow; return; }
    if (cst == C_VARREDURA) {
      if (!clearActive) { clearActive = true; clearSince = now; }
      if (now - clearSince >= (uint32_t)P.clear_ms) {
        cst = C_FRENTE; clearActive = false; gone = true; waitedAfter = false; setWhy(""); *v = P.v_cruise;
      } else *v = P.v_slow;
      return;
    }
    gone = true; waitedAfter = false;                    // em FRENTE o caminho esta livre
    *v = P.v_cruise;
    return;
  }
  uint32_t el = now - tSub;                              // ---- DESVIO ----
  switch (sub) {
    case S_ESPERA:
      if ((int32_t)(now - waitUntil) >= 0) {
        int cls = centerClass(now);
        // a condicao que levou ao desvio persiste? sensor perto/suspeito, ou (desvio pedido pelo celular) a visao
        // segue vendo o centro bloqueado / o celular pediu de novo durante a espera, sem o sensor refutar.
        bool phonePersist = avoidFromPhone && !sensorRefutes(now) &&
                            (avoidAgain || (visUsable(now) && visS[2] < (int)P.vis_free_min));
        if ((cls == R_VALID && centerCm < P.d_stop_cm) || cls == R_SUSPECT || phonePersist) {   // manobra COMPLETA
          waitedAfter = true;
          enterAvoid(now, "fim da espera: obstaculo segue", false, true);
        } else { cst = C_VARREDURA; sub = S_NONE; clearActive = false; setWhy("fim da espera"); *v = P.v_slow; }
      }
      return;
    case S_PARADA:
      if (el >= (uint32_t)P.stop_ms) {
        buildScan(now);
        aimTo(scanDeg_[scanOrd[0]], now);
        goSub(S_OLHA, now);
      }
      return;
    case S_OLHA: {
      int pi = scanOrd[scanIdx];
      bool got = meas.valid && absf(meas.angle - servoDeg) < 0.5f;
      if (got || el >= (uint32_t)P.look_timeout_ms) {
        float sc = (got && meas.cm >= 0) ? meas.cm : noEchoScore(scanDeg_[pi], now);   // sem eco: visao do setor, senao none_score
        scanSc[pi] = sc;
        logEv(1, servoDeg, got ? meas.cm : -1.0f, got ? 1 : 0, now);   // leitura (ou timeout) de cada olhada
        bool early = hintFirst && scanIdx == 0 && sc >= P.d_clear_cm;  // sugestao do celular com folga e sem veto do sensor
        scanIdx++;
        if (early) decideAngle(now, scanDeg_[pi]);
        else if (scanIdx >= scanNPos) decideAngle(now, NAN);
        else { aimTo(scanDeg_[scanOrd[scanIdx]], now); goSub(S_OLHA, now); }
      }
      return;
    }
    case S_RE:
      *v = -P.v_rev;
      if (el >= (uint32_t)revMs) {
        if (turnTarget == 0) { hintOk = false; buildScan(now); aimTo(scanDeg_[scanOrd[0]], now); goSub(S_OLHA, now); }  // sem saida: olha de novo
        else startTurn(now);
      }
      return;
    case S_CURVA: {
      float dir = turnTarget > 0 ? 1.0f : -1.0f, absT = absf(turnTarget);
      float tdps = P.turn_dps > 1 ? P.turn_dps : 1;
      float k = 1.0f;
      bool done, yawOk = turnYaw && yawFresh(now);
      if (yawOk) {                                       // gira ate |delta psi| >= alvo - giro*lead
        float prog = (yawPsiNow(now) - psi0) * dir;      // graus ja girados no sentido do alvo (psi extrapolado)
        float lead = yawR * dir;
        if (lead < 0) lead = 0;
        done = prog >= absT - lead * P.yaw_lead_ms / 1000.0f;
        float slow = P.turn_slow_deg > 0.1f ? P.turn_slow_deg : 0.1f;
        k = clampf((absT - prog) / slow, clampf(P.turn_k_min, 0.05f, 1.0f), 1.0f);   // perto do alvo: gira mais devagar
      } else {
        done = el >= (uint32_t)(absT / tdps * 1000.0f);  // sem yaw: turn_ms = alvo / turn_dps (velocidade cheia)
      }
      *v = P.v_turn * k; *s = dir * k;
      float tout = 1.5f * absT / tdps * 1000.0f + 300.0f;
      if (turnYaw) tout += P.turn_slow_deg / (tdps * clampf(P.turn_k_min, 0.05f, 1.0f)) * 1000.0f;   // a desaceleracao demora
      if (!done && el >= (uint32_t)tout) done = true;    // timeout
      if (done) { turnEnd(now); cst = C_VARREDURA; sub = S_NONE; clearActive = false; setWhy("pos-desvio"); *v = P.v_slow; *s = 0; }
      return;
    }
    case S_PRESO: {                                       // preso: sai quando o centro libera
      aimTo(0, now);
      int cls = centerClass(now);
      bool ok = cls == R_FREE || (cls == R_VALID && centerCm >= P.d_clear_cm);
      if (!ok) clearActive = false;
      else {
        if (!clearActive) { clearActive = true; clearSince = now; }
        if (now - clearSince >= (uint32_t)P.clear_ms) { cst = C_VARREDURA; sub = S_NONE; clearActive = false; setWhy("liberado"); }
      }
      return;
    }
    default: return;
  }
}

static void phoneStep(uint32_t now, float* v, float* s) {
  bool fresh = now - lastDriveT <= (uint32_t)P.wd_ms;
  if (avoidReq) {                                          // celular pediu desvio (com angulo sugerido, se veio)
    avoidReq = false;
    if (sensorRefutes(now)) {                              // visao refutada: sensor le o caminho livre (>= d_clear)
      snprintf(why, sizeof(why), "visao refutada pelo sensor (%.0f cm)", centerCm);
      if (now - tRefuteLog >= 1000) {
        tRefuteLog = now;
        emitf("{\"t\":\"log\",\"msg\":\"avoid recusado: visao refutada pelo sensor (%.0f cm)\"}", centerCm);
      }
    } else if (!delegated || cst != C_DESVIO) {
      delegated = true;
      cst = C_FRENTE; sub = S_NONE;
      hintOk = avoidHintOk; hintDeg = avoidHintDeg;
      enterAvoid(now, "celular: obstaculo", false, false);
      avoidFromPhone = true;
      emitf("{\"t\":\"log\",\"msg\":\"ESP32 assumiu: desvio pedido\"}");
    } else avoidAgain = true;                              // pediu de novo durante a manobra/ESPERA
  }
  if (delegated) {
    if (cst == C_DESVIO) { autoStep(now, v, s); return; }   // termina a manobra antes de devolver
    if (fresh) {                                           // celular voltou e manobra acabou
      delegated = false;
      aimTo(0, now);
      emitf("{\"t\":\"log\",\"msg\":\"celular retomou o controle\"}");
    } else if (P.fallback_auto > 0.5f) { autoStep(now, v, s); return; }
    else { *v = 0; *s = 0; return; }
  }
  if (!fresh) {                                            // watchdog
    if (P.fallback_auto > 0.5f) {
      delegated = true; resetAuto(now);
      emitf("{\"t\":\"log\",\"msg\":\"sem celular: ESP32 segue so com o sensor\"}");
      autoStep(now, v, s);
    } else { *v = 0; *s = 0; }
    return;
  }
  if (haveC && (int32_t)(now - centerT) <= (int32_t)P.fresh_ms) {
    int cls = centerClass(now);
    // caminho livre (sensor >= d_clear, ou sem eco com a visao livre): o obstaculo do ultimo desvio "sumiu"
    if ((cls == R_VALID && centerCm >= P.d_clear_cm) || (cls == R_FREE && visUsable(now) && visS[2] >= (int)P.vis_free_min)) {
      gone = true; waitedAfter = false;
    }
  }
  if (cmdV > 0.02f && haveC && (int32_t)(now - centerT) <= (int32_t)P.fresh_ms) {   // VETO do sensor (mediana, nao o bruto)
    int cls = centerClass(now);
    bool near = cls == R_VALID && centerCm < P.d_stop_cm;
    if (near || cls == R_SUSPECT) {
      delegated = true; cst = C_FRENTE; sub = S_NONE; hintOk = false; avoidFromPhone = false;
      enterAvoid(now, near ? "veto: sensor perto" : "veto: sem eco suspeito", !near, false);
      emitf("{\"t\":\"log\",\"msg\":\"ESP32 assumiu: veto do sensor\"}");
      autoStep(now, v, s);
      return;
    }
  }
  *v = cmdV; *s = cmdS;
}

// ============================== DIAGNOSTICO =================================
// Testes curtos pedidos pelo visor, com o carro armado. Valem os mesmos reflexos do driveApply (hard stop,
// forwardAllowed, servo parado). Ao fim emite {"t":"diag","teste","L","R","psi","rate_med",...} com o yaw recebido
// (psi = graus girados no teste, + = direita; rate_med = mediana do giro na 2a metade) e o carro fica PARADO
// (diagHold) ate um novo goal "explorar" ou o desarme. Quem olha o carro confere o sentido (visor).
static bool diagIsWheel(int t) { return t == D_RODA_E || t == D_RODA_D; }

static void diagStart(int test, uint32_t now) {
  diagTest = test; diagHold = true; diagT0 = now;
  diagDur = (test == D_DIR || test == D_ESQ) ? 1000 : 2000;
  if (diagIsWheel(test)) {
    int n = wDD > 0 ? (int)((wD1 - wD0) / wDD) + 1 : 1;
    n = n < 1 ? 1 : (n > WSTEP_MAX ? WSTEP_MAX : n);
    diagDur = (uint32_t)(n * wStepMs);
    wStep = -1; wN = 0; wBufN = 0; wDuty = 0;
  }
  diagHavePsi = false; diagN = 0; diagNLR = 0; diagSumL = diagSumR = 0; diagYawT = yawT;
  holdNoCorr = (test == D_RETO);                         // "reto" mede o carro cru: sem PI e sem trim
  delegated = false; avoidReq = false;
  cst = C_FRENTE; sub = S_NONE; aimTo(0, now);
  snprintf(why, sizeof(why), "diagnostico: %s", DIAG_N[test]);
}

static int cmpf(const void* a, const void* b) { float x = *(const float*)a, y = *(const float*)b; return x < y ? -1 : (x > y); }

static float medianOf(float* a, int n) {
  qsort(a, n, sizeof(float), cmpf);
  return a[n / 2];
}

static void wheelCloseStep() {                           // fecha o degrau atual: mediana do giro (NAN = sem yaw)
  if (wStep < 0 || wN >= WSTEP_MAX) return;
  wDuties[wN] = wDuty;
  wRates[wN++] = wBufN > 0 ? medianOf(wBuf, wBufN) : NAN;
  wBufN = 0;
}

// {"t":"diag","teste":"roda_esq","duty":[110,125,..],"rate":[0.0,3.1,..],"partida":140,"psi":..,"ms":..}
// partida = 1o duty com |giro| >= 15 graus/s (a roda comecou a empurrar o carro); null = nao partiu
static void wheelFinish(uint32_t now, const char* abort, const char* psi) {
  char du[WSTEP_MAX * 5 + 4], ra[WSTEP_MAX * 8 + 4], pa[12];
  int pu = 0, pr = 0, start = -1;
  du[0] = ra[0] = 0;
  for (int i = 0; i < wN; i++) {
    pu += snprintf(du + pu, sizeof(du) - pu, "%s%d", i ? "," : "", wDuties[i]);
    if (isnan(wRates[i])) pr += snprintf(ra + pr, sizeof(ra) - pr, "%snull", i ? "," : "");
    else pr += snprintf(ra + pr, sizeof(ra) - pr, "%s%.1f", i ? "," : "", wRates[i]);
    if (start < 0 && !isnan(wRates[i]) && absf(wRates[i]) >= 15.0f) start = wDuties[i];
  }
  if (start >= 0) snprintf(pa, sizeof(pa), "%d", start); else strcpy(pa, "null");
  emitf("{\"t\":\"diag\",\"teste\":\"%s\",\"duty\":[%s],\"rate\":[%s],\"partida\":%s,\"psi\":%s,\"ms\":%lu%s%s%s}",
        DIAG_N[diagTest], du, ra, pa, psi, (unsigned long)(now - diagT0),
        abort ? ",\"abortado\":\"" : "", abort ? abort : "", abort ? "\"" : "");
}

static void diagFinish(uint32_t now, const char* abort) {
  char psi[16], med[16];
  if (diagHavePsi) snprintf(psi, sizeof(psi), "%.1f", diagPsi1 - diagPsi0); else strcpy(psi, "null");
  if (diagIsWheel(diagTest)) {
    wheelCloseStep();
    wheelFinish(now, abort, psi);
    wDuty = 0; wStep = -1; diagTest = D_NONE; holdNoCorr = false;
    if (abort) snprintf(why, sizeof(why), "diag abortado: %s", abort); else setWhy("diagnostico: parado");
    return;
  }
  if (diagN > 0) { qsort(diagRates, diagN, sizeof(float), cmpf); snprintf(med, sizeof(med), "%.1f", diagRates[diagN / 2]); }
  else strcpy(med, "null");
  float mL = diagNLR ? diagSumL / diagNLR : 0, mR = diagNLR ? diagSumR / diagNLR : 0;
  emitf("{\"t\":\"diag\",\"teste\":\"%s\",\"L\":%.2f,\"R\":%.2f,\"psi\":%s,\"rate_med\":%s,\"n\":%d,\"ms\":%lu,"
        "\"trim\":%.2f%s%s%s}",
        DIAG_N[diagTest], mL, mR, psi, med, diagN, (unsigned long)(now - diagT0), holdTrim,
        abort ? ",\"abortado\":\"" : "", abort ? abort : "", abort ? "\"" : "");
  diagTest = D_NONE; holdNoCorr = false;
  if (abort) snprintf(why, sizeof(why), "diag abortado: %s", abort); else setWhy("diagnostico: parado");
}

static void diagStep(uint32_t now, float* v, float* s) {
  *v = 0; *s = 0;
  if (diagTest == D_NONE) return;                        // diagHold: parado
  uint32_t el = now - diagT0;
  bool fresh = yawFresh(now);
  if (fresh && !diagHavePsi) { diagHavePsi = true; diagPsi0 = yawPsi; }
  if (fresh) diagPsi1 = yawPsi;
  if (diagIsWheel(diagTest)) {
    if (el >= diagDur) { diagFinish(now, NULL); return; }   // fecha o ultimo degrau
    int i = (int)(el / (uint32_t)wStepMs);
    if (i != wStep) { wheelCloseStep(); wStep = i; }
    if (yawT != diagYawT) {                              // amostra nova: conta so nos ultimos 60% do degrau (giro assentado)
      diagYawT = yawT;
      if (fresh && el - (uint32_t)i * (uint32_t)wStepMs >= (uint32_t)(0.4f * wStepMs) && wBufN < 16) wBuf[wBufN++] = yawR;
    }
    wDuty = (int)clampf(wD0 + i * wDD, 0.0f, 255.0f);
    if (el > 300 && !forwardAllowed(now)) diagFinish(now, "hard stop / sem leitura livre");
    return;                                              // v = s = 0: o driveApply escreve so a roda do teste
  }
  if (yawT != diagYawT) {                                // amostra nova de yaw: guarda o giro (2a metade do teste)
    diagYawT = yawT;
    if (el >= diagDur / 2 && diagN < DIAG_MAX) diagRates[diagN++] = yawR;
  }
  if (el >= diagDur / 2) { diagSumL += curL; diagSumR += curR; diagNLR++; }
  if (el >= diagDur) { diagFinish(now, NULL); return; }
  if (diagTest == D_DIR || diagTest == D_ESQ) { *v = 0; *s = diagTest == D_DIR ? 1.0f : -1.0f; return; }
  *v = P.v_cruise; *s = 0;
  if (el > 300 && !forwardAllowed(now)) diagFinish(now, "hard stop / sem leitura livre");
}

// ============================ ARMAR / BOTAO / LED ===========================
static void setMode(int m, uint32_t now) {                 // idempotente: o mesmo modo nao reinicia a FSM
  if (m == mode && !diagHold) return;
  diagTest = D_NONE; diagHold = false; holdNoCorr = false; wDuty = 0;   // goal explorar depois do diagnostico: volta a andar
  mode = m;
  cmdV = cmdS = 0; delegated = false; resetAuto(now); rtcMode = (uint32_t)mode;
}

// desarma e leva TUDO ao estado inicial: modo padrao, FSM, comandos e RTC (nao retoma depois de um reset)
static void setArmed(bool on, uint32_t now) {
  if (on) { armPending = true; armAt = now + ARM_DELAY_MS; emitf("{\"t\":\"log\",\"msg\":\"armando em %d s\"}", ARM_DELAY_MS / 1000); }
  else {
    armPending = false; armed = false; delegated = false; avoidReq = false; avoidHintOk = false;
    curL = curR = rampL = rampR = 0; cmdV = cmdS = 0;
    if (diagTest != D_NONE) diagFinish(now, "desarmado");
    diagHold = false; turnRepPending = false;
    holdOn = false; holdMode = H_OFF; holdCorr = 0;
    saveTrim();                                            // trim aprendido do rumo -> NVS
    mode = DEFAULT_MODE; resetAuto(now);
    rtcArmed = 0; rtcMode = DEFAULT_MODE; rtcGoal[0] = 0;
    strncpy(netGoal, "parar", sizeof(netGoal) - 1); strncpy(netSrc, "sensor", sizeof(netSrc) - 1);
    emitf("{\"t\":\"goal\",\"g\":\"parar\",\"src\":\"sensor\"}");   // o visor e o celular acompanham o desarme
    emitf("{\"t\":\"log\",\"msg\":\"desarmado\"}");
  }
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
    armPending = false; armed = true; resetAuto(now); delegated = false; rtcArmed = diagHold ? 0 : 1; rtcMode = (uint32_t)mode;
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
  static const char* CN[] = {"FRENTE", "VARREDURA", "DESVIO"};
  static const char* SN[] = {"", "ESPERA", "PARADA", "OLHA", "RE", "CURVA", "PRESO"};
  static const char* KN[] = {"valida", "livre", "suspeito"};
  bool fsm = mode == MODE_AUTO || delegated;              // maquina de estados do ESP32 ativa?
  char ang[16];
  if (haveAng) snprintf(ang, sizeof(ang), "%.0f", chosenAng); else strcpy(ang, "null");
  emitf("{\"t\":\"state\",\"mode\":\"%s\",\"src\":\"%s\",\"armed\":%s,\"esp\":%s,\"st\":\"%s\",\"sub\":\"%s\",\"c\":%.1f,\"deg\":%.1f,"
        "\"cls\":\"%s\",\"ang\":%s,\"vis\":%d,\"yaw\":%d,"
        "\"L\":%.2f,\"R\":%.2f,\"hold\":%d,\"hc\":%.2f,\"trim\":%.2f,\"why\":\"%s\",\"up\":%lu,\"rst\":\"%s\",\"rest\":%lu}",
        mode == MODE_AUTO ? "auto" : "phone", mode == MODE_AUTO ? "sensor" : "visao",
        armed ? "true" : "false", delegated ? "true" : "false",
        diagHold ? "DIAG" : (fsm ? CN[cst] : "-"), diagHold ? (diagTest ? DIAG_N[diagTest] : "parado") : (fsm ? SN[sub] : ""),
        centerCm, servoDeg, haveC ? KN[centerClass(now)] : "-", ang, visUsable(now) ? 1 : 0, yawFresh(now) ? 1 : 0,
        curL, curR, holdMode, holdCorr, holdTrim, why, (unsigned long)(now / 1000),
        (resetReason >= 0 && resetReason <= 10) ? RR[resetReason] : "?", (unsigned long)rtcRestores);
}

static void handleJson(const char* line, uint32_t now) {
  char t[16];
  if (!jsonStr(line, "t", t, sizeof(t))) return;
  float x;
  if (strcmp(t, "aim") == 0) {
    if (mode == MODE_PHONE && !delegated && jsonNum(line, "deg", &x)) aimTo(x, now);
  } else if (strcmp(t, "drive") == 0) {
    float v = 0, s = 0;
    jsonNum(line, "v", &v); jsonNum(line, "s", &s);
    cmdV = v; cmdS = s; lastDriveT = now;
  } else if (strcmp(t, "vis") == 0) {                      // opiniao da visao: ok + 5 setores livres (0..100)
    float ok = 0;
    int s5[5];
    jsonNum(line, "ok", &ok);
    if (ok > 0.5f && jsonInts5(line, "s", s5)) {
      for (int i = 0; i < 5; i++) visS[i] = s5[i] < 0 ? 0 : (s5[i] > 100 ? 100 : s5[i]);
      visOk = true; visT = now;
    } else visOk = false;                                  // ok:0 (calibrando/erro) = sem opiniao
  } else if (strcmp(t, "yaw") == 0) {                      // rumo do giroscopio do celular
    float psi, r = 0;
    if (jsonNum(line, "psi", &psi)) { jsonNum(line, "r", &r); yawPsi = psi; yawR = r; yawT = now; yawHave = true; }
  } else if (strcmp(t, "cfg") == 0) {
    for (int i = 0; i < N_PARAMS; i++)
      if (jsonNum(line, PARAM_TABLE[i].key, &x)) *PARAM_TABLE[i].ptr = x;
    if (jsonFind(line, "pwm_min_l") || jsonFind(line, "pwm_max_l") || jsonFind(line, "pwm_min_r") || jsonFind(line, "pwm_max_r"))
      saveWheels();                                        // faixa por roda: persiste (medida uma vez no teste por roda)
    servoInit = false; aimTo(servoDeg, now);              // reaplica trim do servo
    emitf("{\"t\":\"cfg_ok\"}");
  } else if (strcmp(t, "mode") == 0) {
    char m[12];
    if (jsonStr(line, "m", m, sizeof(m))) setMode(strcmp(m, "phone") == 0 ? MODE_PHONE : MODE_AUTO, now);
  } else if (strcmp(t, "arm") == 0) {
    bool on;
    if (jsonBool(line, "on", &on)) setArmed(on, now);
  } else if (strcmp(t, "ping") == 0) {
    emitf("{\"t\":\"pong\"}");
  } else if (strcmp(t, "diag") == 0) {  // diagnostico do visor: armar | dir | esq | reto | reto_corr | roda_esq | roda_dir | parar | zerar_trim
    char d[16];
    if (!jsonStr(line, "teste", d, sizeof(d))) return;
    if (strcmp(d, "armar") == 0) {                       // arma (4 s) e fica PARADO esperando os testes
      diagHold = true; diagTest = D_NONE; rtcArmed = 0;
      if (!armed && !armPending) setArmed(true, now);
      return;
    }
    if (strcmp(d, "zerar_trim") == 0) {
      holdTrim = 0; saveTrim();
      emitf("{\"t\":\"diag\",\"teste\":\"zerar_trim\",\"trim\":0.00}");
      return;
    }
    if (strcmp(d, "parar") == 0) { if (diagTest != D_NONE) diagFinish(now, "parar"); diagHold = armed || armPending; return; }
    int k = D_NONE;
    for (int i = 1; i <= D_RODA_D; i++) if (strcmp(d, DIAG_N[i]) == 0) k = i;
    if (k == D_NONE) return;
    if (!armed) { emitf("{\"t\":\"diag\",\"teste\":\"%s\",\"erro\":\"arme antes (Armar parado)\"}", d); return; }
    if (diagIsWheel(k)) {                                // degraus opcionais: {"d0":110,"d1":230,"dd":15,"step_ms":300}
      if (jsonNum(line, "d0", &x)) wD0 = clampf(x, 0, 255);
      if (jsonNum(line, "d1", &x)) wD1 = clampf(x, 0, 255);
      if (jsonNum(line, "dd", &x)) wDD = clampf(x, 1, 255);
      if (jsonNum(line, "step_ms", &x)) wStepMs = clampf(x, 150, 2000);
      if (wD1 < wD0) wD1 = wD0;
    }
    rtcArmed = 0;                                        // reset no meio do diagnostico nao retoma a exploracao
    diagStart(k, now);
  } else if (strcmp(t, "trim") == 0) {         // {"t":"trim","v":0}: define (e grava) o trim aprendido do rumo
    if (jsonNum(line, "v", &x)) {
      holdTrim = clampf(x, -P.trim_max, P.trim_max); saveTrim();
      emitf("{\"t\":\"log\",\"msg\":\"trim do rumo = %.3f\"}", holdTrim);
    }
  } else if (strcmp(t, "avoid") == 0) {        // celular viu obstaculo: o ESP32 faz o desvio (deg = angulo sugerido)
    if (mode == MODE_PHONE && armed) {
      avoidReq = true;
      avoidHintOk = jsonNum(line, "deg", &avoidHintDeg);
    }
  } else if (strcmp(t, "goal") == 0) {         // motivacao vinda do visor: {"g":"explorar","src":"sensor"|"visao"} | "parar"
    char g[24], src[8];
    if (jsonStr(line, "g", g, sizeof(g))) {
      if (strcmp(g, "parar") == 0) {
        setArmed(false, now);                  // reset completo (inclui rtcGoal) e avisa os clientes
        return;
      }
      if (!jsonStr(line, "src", src, sizeof(src)) || (strcmp(src, "visao") != 0)) strcpy(src, "sensor");
      strncpy(netGoal, g, sizeof(netGoal) - 1); netGoal[sizeof(netGoal) - 1] = 0;
      strncpy(netSrc, src, sizeof(netSrc) - 1); netSrc[sizeof(netSrc) - 1] = 0;
      if (strcmp(g, "explorar") == 0) {
        strncpy(rtcGoal, g, sizeof(rtcGoal) - 1); rtcGoal[sizeof(rtcGoal) - 1] = 0;
        setMode(strcmp(src, "visao") == 0 ? MODE_PHONE : MODE_AUTO, now);   // a fonte define o modo
        if (!armed && !armPending) setArmed(true, now);
      }
    }
    netRelay(line);
  } else if (strcmp(t, "think") == 0) {        // pensamento do cerebro -> visores
    netRelay(line);
  } else if (strcmp(t, "calibrar") == 0) {     // visor (Explorar) -> cerebro reaprende a cor do chao
    netRelay(line);
  }
}

static void handleText(char* line, uint32_t now) {         // comandos digitaveis no Monitor Serial
  char a[24];
  float x = 0, y = 0;
  if (strcmp(line, "go") == 0) setArmed(true, now);
  else if (strcmp(line, "stop") == 0) setArmed(false, now);
  else if (sscanf(line, "mode %23s", a) == 1) {
    setMode(strcmp(a, "phone") == 0 ? MODE_PHONE : MODE_AUTO, now);
    Serial.printf("modo = %s\n", a);
  } else if (sscanf(line, "set %23s %f", a, &x) == 2) {
    bool ok = false;
    for (int i = 0; i < N_PARAMS; i++)
      if (strcmp(a, PARAM_TABLE[i].key) == 0) { *PARAM_TABLE[i].ptr = x; ok = true; }
    if (ok) { servoInit = false; aimTo(servoDeg, now); }   // reaplica trim do servo
    Serial.printf(ok ? "%s = %.3f\n" : "parametro desconhecido: %s (%.3f)\n", a, x);
  } else if (strcmp(line, "get") == 0) {
    for (int i = 0; i < N_PARAMS; i++) Serial.printf("%s = %.3f\n", PARAM_TABLE[i].key, *PARAM_TABLE[i].ptr);
  } else if (sscanf(line, "aim %f", &x) == 1) { aimTo(x, now); Serial.printf("aim %.1f\n", x); }
  else if (sscanf(line, "drive %f %f", &x, &y) == 2) { cmdV = x; cmdS = y; lastDriveT = now; Serial.printf("drive %.2f %.2f\n", x, y); }
  else if (strcmp(line, "status") == 0) { tHeartbeat = 0; printHeartbeat(now); }
  else if (sscanf(line, "trim %f", &x) == 1) { holdTrim = clampf(x, -P.trim_max, P.trim_max); saveTrim(); Serial.printf("trim = %.3f\n", holdTrim); }
  else if (strcmp(line, "trim") == 0) Serial.printf("trim = %.3f (gravado %.3f)\n", holdTrim, holdTrimSaved);
  else if (strcmp(line, "avoid") == 0) { if (mode == MODE_PHONE && armed) { avoidReq = true; avoidHintOk = false; } }
  else if (strcmp(line, "help") == 0)
    Serial.println("go | stop | mode auto|phone | set <chave> <valor> | get | aim <graus> | drive <v> <s> | avoid | status | trim [valor]");
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
  prefs.begin("carro", true);                              // trim aprendido do rumo (0 na 1a vez)
  holdTrim = holdTrimSaved = clampf(prefs.getFloat("trim", 0.0f), -P.trim_max, P.trim_max);
  P.pwm_min_l = prefs.getFloat("pmin_l", P.pwm_min_l); P.pwm_max_l = prefs.getFloat("pmax_l", P.pwm_max_l);   // faixa por roda
  P.pwm_min_r = prefs.getFloat("pmin_r", P.pwm_min_r); P.pwm_max_r = prefs.getFloat("pmax_r", P.pwm_max_r);
  prefs.end();
  uint32_t now = millis();
  aimTo(0, now);
  lastDriveT = now; tLastApply = now;
  driveApply(now, 0, 0);
  netBegin();
  resetReason = (int)esp_reset_reason();
  bool warm = resetReason != ESP_RST_POWERON && resetReason != ESP_RST_EXT && rtcMagic == RTC_MAGIC;
  if (!warm) { rtcMagic = RTC_MAGIC; rtcArmed = 0; rtcMode = DEFAULT_MODE; rtcRestores = 0; rtcGoal[0] = 0;
               rtcEvSeq = 0; rtcBoot = 0; memset(rtcEv, 0, sizeof(rtcEv)); }
  rtcBoot++;
  emitf("{\"t\":\"hello\",\"fw\":\"%s\",\"mode\":\"%s\",\"armed\":false,\"motors\":%d,\"reset\":\"%s\"}",
        FW_VERSION, mode == MODE_AUTO ? "auto" : "phone", MOTORS_ENABLED,
        (resetReason >= 0 && resetReason <= 10) ? RR[resetReason] : "?");
  if (warm && rtcArmed && resetReason == ESP_RST_BROWNOUT) {  // queda de tensao: NAO retoma (religar os motores repete a queda)
    rtcArmed = 0;
    emitf("{\"t\":\"log\",\"msg\":\"brownout: fica desarmado (nao retoma)\"}");
  } else if (warm && rtcArmed && rtcRestores < MAX_RESTORES) {     // outro reset inesperado andando: retoma
    rtcRestores++;
    mode = rtcMode == MODE_PHONE ? MODE_PHONE : MODE_AUTO;
    if (rtcGoal[0]) { strncpy(netGoal, rtcGoal, sizeof(netGoal) - 1); }
    strncpy(netSrc, mode == MODE_PHONE ? "visao" : "sensor", sizeof(netSrc) - 1);
    emitf("{\"t\":\"log\",\"msg\":\"reset inesperado (%s): retomando %d/%d\"}",
          RR[resetReason <= 10 ? resetReason : 0], (int)rtcRestores, MAX_RESTORES);
    setArmed(true, now);
  } else if (warm && rtcArmed) {
    rtcArmed = 0;
    emitf("{\"t\":\"log\",\"msg\":\"muitos resets seguidos: fica parado\"}");
  }
}

void loop() {
  pollSerial(millis());
  netLoop();
  uint32_t now = millis();   // depois do netLoop: nenhum timestamp (ex.: lastDriveT) fica a frente de `now`
  if (rtcRestores && now > 30000) rtcRestores = 0;       // 30 s sem cair: zera o contador de retomadas
  buttonTask(now);
  servoTick(now);
  sensorTask(now);
  float v = 0, s = 0;
  if (armed) {
    if (diagHold) diagStep(now, &v, &s);                 // diagnostico (ou parado depois dele): FSM suspensa
    else if (mode == MODE_AUTO) autoStep(now, &v, &s);
    else phoneStep(now, &v, &s);
  }
  driveApply(now, v, s);
  turnReportTask(now);
  printHeartbeat(now);
  ledTask(now);
}
