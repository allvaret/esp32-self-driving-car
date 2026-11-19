// Stub de WebSocketsServer.h para o simulador host: clientes simulados (g_ws_clients) e saida "WS ..." no stdout.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <functional>
enum WStype_t { WStype_DISCONNECTED, WStype_CONNECTED, WStype_TEXT };
extern int g_ws_clients;
extern unsigned millis_for_ws();
struct WebSocketsServer {
  explicit WebSocketsServer(int) {}
  std::function<void(uint8_t, WStype_t, uint8_t*, size_t)> cb;
  void begin() {}
  void onEvent(std::function<void(uint8_t, WStype_t, uint8_t*, size_t)> f) { cb = f; }
  void loop() {}
  int connectedClients() { return g_ws_clients; }
  void broadcastTXT(const char* s, size_t n) { printf("WSB %u %.*s\n", millis_for_ws(), (int)n, s); }
  void broadcastTXT(const char* s) { broadcastTXT(s, strlen(s)); }
  void sendTXT(uint8_t num, const char* s) { printf("WSC %u %u %s\n", millis_for_ws(), (unsigned)num, s); }
};
