// Stub da Preferences (NVS do ESP32) para o simulador: chave -> float em memoria (g_nvs).
// sim.cpp pre-carrega com --nvs <chave> <valor> e imprime "NVS <chave>=<valor>" ao final.
#pragma once
#include <map>
#include <string>
extern std::map<std::string, float> g_nvs;
extern int g_nvs_writes;
class Preferences {
 public:
  bool begin(const char* ns, bool readOnly = false) { (void)ns; ro_ = readOnly; return true; }
  void end() {}
  float getFloat(const char* key, float def = 0) {
    auto it = g_nvs.find(key);
    return it == g_nvs.end() ? def : it->second;
  }
  size_t putFloat(const char* key, float v) {
    if (ro_) return 0;
    g_nvs[key] = v; g_nvs_writes++;
    return sizeof(float);
  }
 private:
  bool ro_ = false;
};
