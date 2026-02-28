#pragma once
#include <Arduino.h>

class Blinker {
public:
  explicit Blinker(uint8_t pin, bool activeLow=true);

  void begin();
  void start(int blinkCount, uint32_t onMs, uint32_t offMs);
  void tick(); // викликати завжди

  bool active() const { return _active; }

private:
  uint8_t _pin;
  bool _activeLow;

  bool _active=false;
  int _togglesLeft=0;
  uint32_t _onMs=0, _offMs=0;
  uint32_t _lastToggle=0;
  bool _ledOn=false;

  void writeLed(bool on);
};