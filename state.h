#pragma once
#include <stdint.h>

struct State {
  // timers
  uint32_t lastPollMs  = 0;
  uint32_t lastBlinkMs = 0;

  // current inverter settings (raw 0.1V)
  uint16_t invOffRaw = 0;
  uint16_t invOnRaw  = 0;

  // readings
  float batV = 0;
  float pvV = 0;
  float pvP = 0;
  float mainsV = 0;

  float bypassUv = 0;
  float bypassOv = 0;

  uint16_t activePowerRaw = 0;

  // mode flags
  bool nightMode = false;
  bool acLostMode = false;
  bool acIsLost = false;

  // counters
  int nightDelay = 0;
  int dayDelay = 0;
  int acDelay = 0;
  int acOkStreak = 0;
};