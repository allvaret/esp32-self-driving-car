// Stub de WebServer.h para o simulador host.
#pragma once
#include <functional>
struct WebServer {
  explicit WebServer(int) {}
  void on(const char*, std::function<void()>) {}
  void begin() {}
  void handleClient() {}
  void send(int, const char*, const char*) {}
};
