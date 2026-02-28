#pragma once
#include <ESP8266WiFi.h>

class TcpLink {
public:
  TcpLink(WiFiClient& c, IPAddress ip, uint16_t port);
  bool ensure(uint32_t intervalMs=3000);
  bool connected() const { return _c.connected(); }
  void drop() { _c.stop(); }

private:
  WiFiClient& _c;
  IPAddress _ip;
  uint16_t _port;
  uint32_t _lastTry=0;
};