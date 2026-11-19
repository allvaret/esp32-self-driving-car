// Stub de WiFi.h para o simulador host.
#pragma once
#include <string>
#include <cstdint>
#define WIFI_AP 2
struct IPStub { std::string toString() const { return "192.168.4.1"; } };
struct WiFiClass {
  void mode(int) {}
  bool softAP(const char*, const char*) { return true; }
  void setSleep(bool) {}
  IPStub softAPIP() { return IPStub(); }
};
extern WiFiClass WiFi;
