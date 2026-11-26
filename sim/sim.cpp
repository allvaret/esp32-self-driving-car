// Simulador host do car_firmware.ino: relogio virtual, sensor HC-SR04 simulado, serial por script.
// Mundo: 5 posicoes por angulo (-30,-15,0,+15,+30 graus). Yaw simulado a partir das rodas (--world-dps, --yaw).
#include "Arduino.h"
#include <cmath>
#if __has_include("../car_firmware/car_firmware.ino")
#include "../car_firmware/car_firmware.ino"   // layout sim/ + car_firmware/
#elif __has_include("car_firmware.ino")
#include "car_firmware.ino"                    // arquivos soltos na mesma pasta
#else
#error "Coloque car_firmware.ino em ../car_firmware/ ou nesta mesma pasta"
#endif
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

uint64_t g_us = 0;
std::map<std::string, float> g_nvs;
int g_nvs_writes = 0;
WiFiClass WiFi;
int g_ws_clients = 0;
unsigned millis_for_ws() { return millis(); }
esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
static int g_pin[64];
static int g_pwm_pin[64];       // duty por pino (nos dois cores), bruto
static int g_bits[64];          // resolucao do PWM por pino (0 = nao configurado: 8)
static int g_chbits[16];
static int g_ch2pin[16];
static uint32_t g_hz[64];       // frequencia do PWM por pino (0 = sem PWM)
static uint32_t g_chhz[16];
static float duty8(int pin) {   // duty na escala 0..255, qualquer resolucao
  int b = g_bits[pin] ? g_bits[pin] : 8;
  return (float)g_pwm_pin[pin] * 255.0f / (float)((1 << b) - 1);
}
static int g_btn = HIGH;
static std::deque<char> g_in;
static std::string g_outbuf;

// ---- mundo simulado ----
static float W_bin[5] = {300, 300, 300, 300, 300};        // cm em -30,-15,0,+15,+30 (estatico, preso ao carro)
static float W_obst_cm = 300, W_t_obst = 1e12, W_t_clear = 1e12;   // obstaculo temporario no centro
static float W_none_t0 = 1e12, W_none_t1 = 1e12, W_far_t0 = 1e12, W_far_t1 = 1e12;
static float W_dps = 90.0f;                                // giro real do carro com (uE-uD)=1,2 (v_turn 0,3 + s=1)
static bool W_yaw = false;                                 // injeta {"t":"yaw"} a cada 50 ms
static double W_psi = 0, W_rate = 0;
static float W_uL = 0, W_uR = 0;                           // velocidade de cada roda FISICA (normalizada)
static float W_lag = 0;                                    // atraso (ms) do {"t":"yaw"} ate chegar ao ESP32 (--yaw-lag)
static float W_tau = 0;                                    // inercia (ms): o giro segue as rodas com 1a ordem (--tau)

// ---- fiacao FISICA do carro (medida pelo giroscopio nos logs de 20/11 e 21/11; NAO vem do firmware) ----
// ENA/IN1/IN2 (OUT1/OUT2) = roda dianteira ESQUERDA; ENB/IN3/IN4 (OUT3/OUT4) = roda dianteira DIREITA.
// "Para frente" em cada roda = IN_a LOW / IN_b HIGH (o sentido ja foi acertado no carro com o firmware atual).
// Velocidade da roda a partir do PWM: u = ef * max(0, duty - zm) / (pwm_max - pwm_min).
// Padrao ef = 1 e zona morta zm = pwm_min: u = |comando| (carro ideal). --wheel-left/right ef zm muda isso.
// --nopwm-left: a roda esquerda IGNORA o PWM (EN preso em 1): com sentido dado, gira como em duty 255.
static float W_effL = 1, W_effR = 1, W_dzL = NAN, W_dzR = NAN;
static bool W_noPwmL = false;
static float wheelU(int en, int ina, int inb, float eff, float dz, bool noPwm = false) {
  float duty = noPwm ? 255.0f : duty8(en);
  int dir = (g_pin[ina] == LOW && g_pin[inb] == HIGH) ? 1 : ((g_pin[ina] == HIGH && g_pin[inb] == LOW) ? -1 : 0);
  if (duty <= 0 || dir == 0) return 0;
  float z = std::isnan(dz) ? P.pwm_min : dz;
  float span = P.pwm_max - P.pwm_min > 1 ? P.pwm_max - P.pwm_min : 1;
  float u = eff * (duty - z) / span;
  return dir * (u > 0 ? u : 0);
}

// --deadend DEG: beco. A partir de --obst T CM TODAS as direcoes leem CM (parede em volta) ate o carro girar DEG graus
// (W_psi, o rumo fisico) em relacao ao rumo de quando o beco apareceu; dai em diante o caminho fica aberto (300 cm).
static float W_dead_deg = 0;
static bool W_dead_have = false, W_dead_open = false;
static double W_dead_psi0 = 0;
static float simCm(float deg, uint32_t ms) {
  if (W_dead_deg > 0 && ms >= W_t_obst) {
    if (!W_dead_have) { W_dead_have = true; W_dead_psi0 = W_psi; }
    if (!W_dead_open && std::fabs(W_psi - W_dead_psi0) >= W_dead_deg) W_dead_open = true;
    return W_dead_open ? 300.0f : W_obst_cm;
  }
  int b = (int)lroundf(deg / 15.0f) + 2;
  b = b < 0 ? 0 : (b > 4 ? 4 : b);
  float cm = W_bin[b];
  if (b == 2 && ms >= W_t_obst && ms < W_t_clear) cm = W_obst_cm;
  return cm;
}

// ---- BATIDA e camera do celular (0.7.0) ----
// --bump T DEG: aos T ms o carro encosta num obstaculo que o SONAR NAO VE (objeto baixo). Enquanto o rumo fisico estiver a
// menos de DEG graus do rumo da batida, andar para a frente nao avanca: a folga ate ele (gap) cresce com a re e cai com a
// frente (30 cm/s por unidade de roda); com gap 0 a frente fica presa. Girar DEG ou mais = obstaculo fora do caminho.
// --cam: injeta {"t":"vis","ok":0,...,"m":M} a cada 100 ms, M = movimento FISICO da imagem (0,8 parado/preso + 25 x
// avanco real + 0,15 x |giro|). --cam-noise X troca o 0,8 do parado (vibracao com as rodas patinando).
static float W_bumpT = 1e12, W_bumpDeg = 0, W_camNoise = 0.8f;
static bool W_cam = false, W_bumpHave = false, W_bumpGone = false;
static double W_bumpPsi = 0, W_gap = 0;
static int W_bumps = 0;                    // vezes que o carro ficou preso (gap chegou a 0 com a frente empurrando)
static bool W_stuck = false;
static float physFwd(double dt, uint32_t ms) {          // avanco REAL (normalizado) depois do obstaculo
  float u = (W_uL + W_uR) * 0.5f;
  if (ms < W_bumpT || W_bumpGone) return u;
  if (!W_bumpHave) { W_bumpHave = true; W_bumpPsi = W_psi; W_gap = 0; }
  if (std::fabs(W_psi - W_bumpPsi) >= W_bumpDeg) { W_bumpGone = true; return u; }
  W_gap -= u * 30.0 * dt;
  if (W_gap > 0) { W_stuck = false; return u; }
  W_gap = 0;
  if (u > 0.02f && !W_stuck) { W_stuck = true; W_bumps++; }
  return u > 0 ? 0.0f : u;
}

void pinMode(int, int) {}
void digitalWrite(int p, int v) { g_pin[p] = v; }
int digitalRead(int p) { return p == PIN_BTN ? g_btn : g_pin[p]; }
// ---- servo do sensor FISICO: o angulo sai do PWM no pino do servo (50 Hz, 16 bits), NAO da variavel do firmware ----
// --servo-dead: o servo nao gira (fio/alimentacao): o sensor fica sempre a 0 grau, com PWM normal no pino.
// --servo-nopwm: o PWM do pino do servo nao configura (ledcAttach falha).
static bool W_servoDead = false, W_servoNoPwm = false;
static float physServoDeg() {
  if (W_servoDead || g_hz[PIN_SENSOR_SERVO] == 0) return 0.0f;
  float us = (float)g_pwm_pin[PIN_SENSOR_SERVO] * 20000.0f / 65535.0f;
  if (us < 400.0f) return 0.0f;                                         // sem pulso: o servo nao sai do lugar
  return SENSOR_SERVO_SIGN * (us - SERVO_CENTER_US) / SERVO_US_PER_DEG;
}
unsigned long pulseIn(int, int, unsigned long timeout) {
  uint32_t ms = millis();
  float cm = simCm(physServoDeg(), ms);
  if (ms >= W_none_t0 && ms < W_none_t1) cm = 1e9;
  if (ms >= W_far_t0 && ms < W_far_t1) cm = 1e9;                      // objeto "invisivel" (perna fina)
  float us = cm * 58.3f;
  if (us > (float)timeout) { g_us += timeout; return 0; }
  g_us += (uint64_t)us;
  return (unsigned long)us;
}
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t hz, uint8_t bits) {
  if (W_servoNoPwm && pin == PIN_SENSOR_SERVO) return false;
  g_bits[pin] = bits; g_hz[pin] = hz; return true;
}
bool ledcAttachChannel(uint8_t pin, uint32_t hz, uint8_t bits, uint8_t) { return ledcAttach(pin, hz, bits); }
bool ledcWrite(uint8_t pin, uint32_t duty) { if (!g_hz[pin]) return false; g_pwm_pin[pin] = (int)duty; return true; }
uint32_t ledcReadFreq(uint8_t pin) { return g_pwm_pin[pin] ? g_hz[pin] : 0; }   // como o core 3.3: duty 0 -> 0
#else
double ledcSetup(uint8_t ch, double hz, uint8_t bits) { g_chbits[ch] = bits; g_chhz[ch] = (uint32_t)hz; return hz; }
void ledcAttachPin(uint8_t pin, uint8_t ch) {
  if (W_servoNoPwm && pin == PIN_SENSOR_SERVO) return;
  g_ch2pin[ch] = pin; g_bits[pin] = g_chbits[ch]; g_hz[pin] = g_chhz[ch];
}
void ledcWrite(uint8_t ch, uint32_t duty) { g_pwm_pin[g_ch2pin[ch]] = (int)duty; }
double ledcReadFreq(uint8_t ch) { return g_chhz[ch]; }
#endif
int SerialStub::available() { return (int)g_in.size(); }
int SerialStub::read() { if (g_in.empty()) return -1; char c = g_in.front(); g_in.pop_front(); return (unsigned char)c; }
static void emit(const char* s) {
  g_outbuf += s;
  size_t p;
  while ((p = g_outbuf.find('\n')) != std::string::npos) {
    printf("OUT %u %s\n", millis(), g_outbuf.substr(0, p).c_str());
    g_outbuf.erase(0, p + 1);
  }
}
void SerialStub::print(const char* s) { emit(s); }
void SerialStub::println(const char* s) { emit(s); emit("\n"); }
size_t SerialStub::write(const uint8_t* b, size_t n) { std::string t((const char*)b, n); emit(t.c_str()); return n; }
size_t SerialStub::write(char c) { char t[2] = {c, 0}; emit(t); return 1; }
int SerialStub::printf(const char* fmt, ...) {
  char b[512]; va_list a; va_start(a, fmt); int n = vsnprintf(b, sizeof b, fmt, a); va_end(a); emit(b); return n;
}
SerialStub Serial;

static const char* SUBN[] = {"-", "ESPERA", "PARADA", "OLHA", "RE", "CURVA", "PRESO"};

int main(int argc, char** argv) {
  uint32_t t_end = 20000; const char* inFile = NULL;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto f = [&](float& x) { x = (float)atof(argv[++i]); };
    if (a == "--t-end") t_end = (uint32_t)atoi(argv[++i]);
    else if (a == "--obst") { f(W_t_obst); f(W_obst_cm); }
    else if (a == "--clear") f(W_t_clear);
    else if (a == "--deadend") f(W_dead_deg);
    else if (a == "--left") { float x; f(x); W_bin[0] = W_bin[1] = x; }
    else if (a == "--right") { float x; f(x); W_bin[3] = W_bin[4] = x; }
    else if (a == "--ang") { float d, x; f(d); f(x); int b = (int)lroundf(d / 15.0f) + 2; if (b >= 0 && b <= 4) W_bin[b] = x; }
    else if (a == "--none") { f(W_none_t0); f(W_none_t1); }
    else if (a == "--far") { f(W_far_t0); f(W_far_t1); }
    else if (a == "--world-dps") f(W_dps);
    else if (a == "--yaw") W_yaw = true;
    else if (a == "--yaw-lag") f(W_lag);
    else if (a == "--tau") f(W_tau);
    else if (a == "--nvs") { std::string k = argv[++i]; g_nvs[k] = (float)atof(argv[++i]); }
    else if (a == "--wheel-left") { f(W_effL); f(W_dzL); }
    else if (a == "--wheel-right") { f(W_effR); f(W_dzR); }
    else if (a == "--nopwm-left") W_noPwmL = true;
    else if (a == "--servo-dead") W_servoDead = true;
    else if (a == "--servo-nopwm") W_servoNoPwm = true;
    else if (a == "--cam") W_cam = true;
    else if (a == "--cam-noise") f(W_camNoise);
    else if (a == "--bump") { f(W_bumpT); f(W_bumpDeg); }      // NAO liga a --cam (sem o celular nao ha deteccao)
    else if (a == "--ws") g_ws_clients = atoi(argv[++i]);
    else if (a == "--input") inFile = argv[++i];
    else if (a == "--set") { std::string k = argv[++i]; float v = (float)atof(argv[++i]);
      for (int j = 0; j < N_PARAMS; j++) if (k == PARAM_TABLE[j].key) *PARAM_TABLE[j].ptr = v; }
  }
  struct In { uint32_t ms; std::string s; }; std::vector<In> script;
  if (inFile) { std::ifstream fi(inFile); std::string line;
    while (std::getline(fi, line)) { size_t tab = line.find('\t'); if (tab == std::string::npos) continue;
      script.push_back({(uint32_t)atoi(line.substr(0, tab).c_str()), line.substr(tab + 1)}); } }
  std::deque<std::pair<uint32_t, std::string>> yawQ;
  size_t si = 0; uint32_t next_tr = 0, next_yaw = 0, next_cam = 0; float fwdNow = 0; int pc = -1, ps = -1, pa = -1; uint64_t last_us = 0;
  setup();
  while (millis() < t_end) {
    uint32_t ms = millis();
    while (si < script.size() && script[si].ms <= ms) {
      const std::string& s = script[si++].s;
      if (s == "@btn_down") g_btn = LOW; else if (s == "@btn_up") g_btn = HIGH;
      else if (s == "@ws_connect") { g_ws_clients++; uint8_t no = (uint8_t)g_ws_clients; wsEvent(no, WStype_CONNECTED, NULL, 0); }
      else { for (char c : s) g_in.push_back(c); g_in.push_back('\n'); }
    }
    g_us += 2000;
    loop();
    double dt = (double)(g_us - last_us) / 1e6; last_us = g_us;
    W_uL = wheelU(PIN_ENA, PIN_IN1, PIN_IN2, W_effL, W_dzL, W_noPwmL);   // roda ESQUERDA (ENA, OUT1/OUT2)
    W_uR = wheelU(PIN_ENB, PIN_IN3, PIN_IN4, W_effR, W_dzR);   // roda DIREITA (ENB, OUT3/OUT4)
    double cmdRate = W_dps * (double)(W_uL - W_uR) / 1.2;  // + = direita (roda esquerda mais rapida)
    W_rate = W_tau > 0 ? W_rate + (cmdRate - W_rate) * (dt * 1000.0) / (W_tau + dt * 1000.0) : cmdRate;
    W_psi += W_rate * dt;
    fwdNow = physFwd(dt, millis());
    if (W_cam && millis() >= next_cam) { next_cam = millis() + 100;
      char b[96]; snprintf(b, sizeof b, "{\"t\":\"vis\",\"ok\":0,\"s\":[0,0,0,0,0],\"m\":%.1f}",
                           W_camNoise + 25.0f * std::fabs(fwdNow) + 0.15f * std::fabs((float)W_rate));
      for (char c : std::string(b)) g_in.push_back(c);
      g_in.push_back('\n'); }
    if (W_yaw && millis() >= next_yaw) { next_yaw = millis() + 50;
      char b[96]; snprintf(b, sizeof b, "{\"t\":\"yaw\",\"psi\":%.2f,\"r\":%.2f}", W_psi, W_rate);
      yawQ.push_back({millis() + (uint32_t)W_lag, b}); }
    while (!yawQ.empty() && yawQ.front().first <= millis()) {
      for (char c : yawQ.front().second) g_in.push_back(c);
      g_in.push_back('\n'); yawQ.pop_front(); }
    if (cst != pc || sub != ps || (int)armed != pa) { printf("EV %u cst=%d sub=%d subn=%s armed=%d psi=%.2f why=%s\n", millis(), cst, sub, SUBN[sub], armed, W_psi, why); pc = cst; ps = sub; pa = armed; }
    if (millis() >= next_tr) { next_tr = millis() + 100;
      printf("TR %u armed=%d mode=%d cst=%d sub=%d subn=%s curL=%.3f curR=%.3f servo=%.1f phys=%.1f center=%.1f cls=%d delegated=%d psi=%.1f "
             "enaPWM=%.0f enbPWM=%.0f in1=%d in2=%d in3=%d in4=%d rtcArmed=%u rtcMode=%u rtcGoal=%s netGoal=%s netSrc=%s ang=%.0f haveAng=%d avoidN=%d guardLevel=%d uL=%.3f uR=%.3f rate=%.1f hold=%d hc=%.3f trim=%.3f cred=%.3f esc=%d fwd=%.3f bumps=%d stl=%d bat=%d\n",
             millis(), armed, mode, cst, sub, SUBN[sub], curL, curR, servoDeg, physServoDeg(), centerCm, haveC ? centerClass(millis()) : -1, (int)delegated, W_psi,
             duty8(PIN_ENA), duty8(PIN_ENB),
             g_pin[PIN_IN1], g_pin[PIN_IN2], g_pin[PIN_IN3], g_pin[PIN_IN4],
             (unsigned)rtcArmed, (unsigned)rtcMode, rtcGoal[0] ? rtcGoal : "-", netGoal, netSrc, chosenAng, (int)haveAng, avoidN, guardLevel, W_uL, W_uR, W_rate, holdMode, holdCorr, holdTrim, credRevMs(millis()) / 1000.0f, escN, fwdNow, W_bumps, (int)stallWatch, stallN); }
  }
  for (auto& kv : g_nvs) printf("NVS %s=%.4f\n", kv.first.c_str(), kv.second);
  printf("NVSW %d\n", g_nvs_writes);
  return 0;
}
