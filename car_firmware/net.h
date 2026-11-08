// net.h - softAP + pagina web (porta 80) + WebSocket (porta 81)            car_firmware 0.2
//
// Rede:  o ESP32 cria a rede "CARRINHO" (senha em secrets.h), IP fixo 192.168.4.1.
// Quem conecta:
//   - celular VISOR  : abre http://192.168.4.1 no navegador (pagina em web_ui.h). Sem app.
//   - celular CEREBRO: S9 (Python) conecta em ws://192.168.4.1:81 e manda drive/aim/think.
// Protocolo: o MESMO JSON da serial (uma mensagem por frame). Tudo que o firmware emite com
// emitf() vai para a serial E para todos os clientes WebSocket.
// Biblioteca: "WebSockets" de Markus Sattler (Links2004)  ->  arduino-cli lib install "WebSockets"
#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <stdarg.h>
#include "web_ui.h"

#define AP_SSID "CARRINHO"
// Senha do AP: crie car_firmware/secrets.h (copie de secrets.example.h; fica fora do git).
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef AP_PASS
#define AP_PASS "troque-esta-senha"   // padrao publico: defina a sua em secrets.h (minimo 8 caracteres)
#endif
#define WS_PORT 81

static WebServer http(80);
static WebSocketsServer ws(WS_PORT);
static bool netUp = false;
static char netGoal[24] = "nenhuma";   // ultima motivacao recebida (reenviada a quem conecta)

static void handleJson(const char* line, uint32_t now);   // definida no .ino

// printf para o protocolo: serial + broadcast WebSocket (sem o '\n' final no WS)
static void emitf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  Serial.write((const uint8_t*)buf, n);
  if (buf[n - 1] != '\n') Serial.write('\n');
  while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = 0;
  if (netUp && n > 0 && ws.connectedClients() > 0) ws.broadcastTXT(buf, n);
}

// repassa uma linha JSON crua (think/goal) para a serial e todos os clientes
static void netRelay(const char* line) {
  Serial.println(line);
  if (netUp && ws.connectedClients() > 0) ws.broadcastTXT(line);
}

static void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    char b[64];
    snprintf(b, sizeof(b), "{\"t\":\"goal\",\"g\":\"%s\"}", netGoal);
    ws.sendTXT(num, b);
  } else if (type == WStype_TEXT) {
    static char buf[384];
    if (len == 0 || len >= sizeof(buf)) return;          // vazio ou longo demais: descarta
    memcpy(buf, payload, len);
    buf[len] = 0;
    if (buf[0] == '{') handleJson(buf, millis());
  }
}

static void netBegin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);                                    // menos latencia
  http.on("/", []() { http.send(200, "text/html", WEB_UI); });
  http.begin();
  ws.begin();
  ws.onEvent(wsEvent);
  netUp = true;
  Serial.printf("{\"t\":\"log\",\"msg\":\"rede %s em %s\"}\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

static void netLoop() {
  if (!netUp) return;
  http.handleClient();
  ws.loop();
}