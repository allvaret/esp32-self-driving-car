// Stub minimo da API Arduino/ESP32 para rodar a LOGICA do firmware no PC (g++). Nao e o core real.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <string>
#include <deque>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
extern uint64_t g_us;
inline uint32_t millis() { return (uint32_t)(g_us / 1000ULL); }
inline uint32_t micros() { return (uint32_t)g_us; }
inline void delay(unsigned ms) { g_us += (uint64_t)ms * 1000ULL; }
inline void delayMicroseconds(unsigned us) { g_us += us; }
void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
unsigned long pulseIn(int pin, int state, unsigned long timeout);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t res);
bool ledcWrite(uint8_t pin, uint32_t duty);
#else
double ledcSetup(uint8_t ch, double freq, uint8_t bits);
void ledcAttachPin(uint8_t pin, uint8_t ch);
void ledcWrite(uint8_t ch, uint32_t duty);
#endif
struct SerialStub {
  void begin(int) {}
  int available();
  int read();
  void print(const char* s);
  void println(const char* s);
  int printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};
extern SerialStub Serial;