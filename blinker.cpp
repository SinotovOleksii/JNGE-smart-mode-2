#include "blinker.h"

Blinker::Blinker(uint8_t pin, bool activeLow): _pin(pin), _activeLow(activeLow) {}

void Blinker::begin() {
  pinMode(_pin, OUTPUT);
  writeLed(false);
}

void Blinker::writeLed(bool on) {
  if (_activeLow) digitalWrite(_pin, on ? LOW : HIGH);
  else           digitalWrite(_pin, on ? HIGH: LOW);
}

void Blinker::start(int blinkCount, uint32_t onMs, uint32_t offMs) {
  if (_active) return;
  _active = true;
  _togglesLeft = blinkCount * 2;
  _onMs = onMs; _offMs = offMs;
  _lastToggle = millis();
  _ledOn = false;
  writeLed(false);
}

void Blinker::tick() {
  if (!_active) return;
  uint32_t now = millis();
  uint32_t interval = _ledOn ? _onMs : _offMs;

  if ((uint32_t)(now - _lastToggle) >= interval) {
    _lastToggle = now;
    _ledOn = !_ledOn;
    writeLed(_ledOn);

    if (--_togglesLeft <= 0) {
      _active = false;
      writeLed(false);
    }
  }
}