#include "tcp_link.h"

TcpLink::TcpLink(WiFiClient& c, IPAddress ip, uint16_t port)
  : _c(c), _ip(ip), _port(port) {}

bool TcpLink::ensure(uint32_t intervalMs) {
  if (_c.connected()) return true;

  uint32_t now = millis();
  if ((uint32_t)(now - _lastTry) < intervalMs) return false;
  _lastTry = now;

  if (WiFi.status() != WL_CONNECTED) return false;

  _c.stop();
  if (!_c.connect(_ip, _port)) return false;

  return true;
}