// Simulador host do car_firmware.ino: relogio virtual, sensor HC-SR04 simulado, serial por script.
#include "Arduino.h"
#if __has_include("../car_firmware/car_firmware.ino")
#include "../car_firmware/car_firmware.ino"   // layout esp32/sim + esp32/car_firmware
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
static int g_pin[64];
static int g_pwm_pin[64];       // duty por pino (nos dois cores)
static int g_ch2pin[16];
static int g_btn = HIGH;
static std::deque<char> g_in;
static std::string g_outbuf;

// ---- mundo simulado ----
static float W_obst_cm = 300, W_left = 300, W_right = 300, W_t_obst = 1e12, W_t_clear = 1e12;
static float W_none_t0 = 1e12, W_none_t1 = 1e12, W_far_t0 = 1e12, W_far_t1 = 1e12;

static float simCm(float deg, uint32_t ms) {
  float cm = deg < -15 ? W_left : (deg > 15 ? W_right : 300.0f);
  if (deg >= -15 && deg <= 15 && ms >= W_t_obst && ms < W_t_clear) cm = W_obst_cm;
  return cm;
}

void pinMode(int, int) {}
void digitalWrite(int p, int v) { g_pin[p] = v; }
int digitalRead(int p) { return p == PIN_BTN ? g_btn : g_pin[p]; }
unsigned long pulseIn(int, int, unsigned long timeout) {
  uint32_t ms = millis();
  float cm = simCm(servoDeg, ms);
  if (ms >= W_none_t0 && ms < W_none_t1) cm = 1e9;
  if (ms >= W_far_t0 && ms < W_far_t1) cm = 1e9;                      // objeto "invisivel" (perna fina)
  float us = cm * 58.3f;
  if (us > (float)timeout) { g_us += timeout; return 0; }
  g_us += (uint64_t)us;
  return (unsigned long)us;
}
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
bool ledcWrite(uint8_t pin, uint32_t duty) { g_pwm_pin[pin] = (int)duty; return true; }
#else
double ledcSetup(uint8_t, double, uint8_t) { return 0; }
void ledcAttachPin(uint8_t pin, uint8_t ch) { g_ch2pin[ch] = pin; }
void ledcWrite(uint8_t ch, uint32_t duty) { g_pwm_pin[g_ch2pin[ch]] = (int)duty; }
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
int SerialStub::printf(const char* fmt, ...) {
  char b[512]; va_list a; va_start(a, fmt); int n = vsnprintf(b, sizeof b, fmt, a); va_end(a); emit(b); return n;
}
SerialStub Serial;

int main(int argc, char** argv) {
  uint32_t t_end = 20000; const char* inFile = NULL;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto f = [&](float& x) { x = (float)atof(argv[++i]); };
    if (a == "--t-end") t_end = (uint32_t)atoi(argv[++i]);
    else if (a == "--obst") { f(W_t_obst); f(W_obst_cm); }
    else if (a == "--clear") f(W_t_clear);
    else if (a == "--left") f(W_left);
    else if (a == "--right") f(W_right);
    else if (a == "--none") { f(W_none_t0); f(W_none_t1); }
    else if (a == "--far") { f(W_far_t0); f(W_far_t1); }
    else if (a == "--input") inFile = argv[++i];
    else if (a == "--set") { std::string k = argv[++i]; float v = (float)atof(argv[++i]);
      for (int j = 0; j < N_PARAMS; j++) if (k == PARAM_TABLE[j].key) *PARAM_TABLE[j].ptr = v; }
  }
  struct In { uint32_t ms; std::string s; }; std::vector<In> script;
  if (inFile) { std::ifstream fi(inFile); std::string line;
    while (std::getline(fi, line)) { size_t tab = line.find('\t'); if (tab == std::string::npos) continue;
      script.push_back({(uint32_t)atoi(line.substr(0, tab).c_str()), line.substr(tab + 1)}); } }
  size_t si = 0; uint32_t next_tr = 0; int pc = -1, ps = -1, pa = -1;
  setup();
  while (millis() < t_end) {
    uint32_t ms = millis();
    while (si < script.size() && script[si].ms <= ms) {
      const std::string& s = script[si++].s;
      if (s == "@btn_down") g_btn = LOW; else if (s == "@btn_up") g_btn = HIGH;
      else { for (char c : s) g_in.push_back(c); g_in.push_back('\n'); }
    }
    g_us += 2000;
    loop();
    if (cst != pc || sub != ps || (int)armed != pa) { printf("EV %u cst=%d sub=%d armed=%d why=%s\n", millis(), cst, sub, armed, why); pc = cst; ps = sub; pa = armed; }
    if (millis() >= next_tr) { next_tr = millis() + 100;
      printf("TR %u armed=%d mode=%d cst=%d sub=%d curL=%.3f curR=%.3f servo=%.1f center=%.1f enaPWM=%d enbPWM=%d in1=%d in2=%d in3=%d in4=%d steerPWM=%d\n",
             millis(), armed, mode, cst, sub, curL, curR, servoDeg, centerCm, g_pwm_pin[PIN_ENA], g_pwm_pin[PIN_ENB],
             g_pin[PIN_IN1], g_pin[PIN_IN2], g_pin[PIN_IN3], g_pin[PIN_IN4], g_pwm_pin[PIN_STEER_SERVO]); }
  }
  return 0;
}