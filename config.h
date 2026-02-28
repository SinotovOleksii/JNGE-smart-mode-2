#pragma once
#include <Arduino.h>
#include <stdint.h>


// WiFi / TCP
//constexpr const char* WIFI_SSID = "EW11_A55A";
constexpr const char* WIFI_SSID = "Alex";
constexpr const char* WIFI_PASS = "INeedConnect";
constexpr uint16_t TCP_PORT = 8899;
constexpr uint8_t  MACHINE_ADDR = 0x06;

// Timers
constexpr uint32_t LOOP_TIMER_MS  = 20000UL;
constexpr uint32_t BLINK_EVERY_MS = 5000UL;

// Battery setup
constexpr int BATTERY_STRING = 2;

// Thresholds
constexpr float MIN_PV_POWER   = 150.0f;
constexpr float MIN_PV_VOLTAGE = 88.0f;
constexpr float MIN_BAT_VOLT   = 13.3f * BATTERY_STRING;

// Mode delays (in loop ticks)
constexpr int START_DAY_DELAY   = 10;
constexpr int START_NIGHT_DELAY = 15;
constexpr int AC_LOST_DELAY     = 3;

// Inverter voltages (raw, 0.1V units)
constexpr uint16_t V_NIGHT_OFF = 130 * BATTERY_STRING;
constexpr uint16_t V_NIGHT_ON  = 150 * BATTERY_STRING;
constexpr uint16_t V_DAY_OFF   = 115 * BATTERY_STRING;
constexpr uint16_t V_DAY_ON    = 131 * BATTERY_STRING;
constexpr uint16_t V_ACLOST_OFF = 111 * BATTERY_STRING;
constexpr uint16_t V_ACLOST_ON  = 125 * BATTERY_STRING;