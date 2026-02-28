#pragma once
#include <Arduino.h>
#include <ESP8266WiFi.h>
//#include <FastCRC.h>
#include "Crc16.h"

class JNGEdevice {
public:
  static const size_t MAX_LAST_RESPONSE = 128;

  explicit JNGEdevice(uint8_t addr, WiFiClient& client);

  // low-level io
  bool readExact(uint8_t* dst, size_t need, uint32_t timeoutMs);
  bool readExactRetry(uint8_t* dst, size_t need, uint32_t timeoutMs, int tries = 2);
  size_t readUntilIdle(uint8_t* dst, size_t cap, uint32_t firstByteTimeoutMs, uint32_t idleGapMs);

  // write command with addr+value+crc, read reply until idle, validate crc+addr, store lastResponse
  bool sendCommand(uint8_t* cmd, size_t cmdSize, uint8_t addr, uint16_t value);

  // param helpers
  uint16_t getParamRaw(const uint8_t* dataArr, int arrSize,
                       uint16_t startAddr, uint16_t paramAddr,
                       const char* caption = nullptr);

  float getParamF(const uint8_t* dataArr, int arrSize,
                  uint16_t startAddr, uint16_t paramAddr,
                  float coefficient, const char* caption = nullptr);

  void printData(const uint8_t* dataArr, int arrSize);

  // crc helpers
  uint16_t calcCRC16(uint8_t* dataArr, int arrSize);
  bool checkCRCresponse(uint8_t* dataArr, int arrSize);

  uint8_t getMachineAddr() const { return machineAddr_; }
  bool connected() const { return client_.connected(); }

  // last response
  uint8_t lastResponse[MAX_LAST_RESPONSE];
  size_t sizeOfResponse = 0;

protected:
  WiFiClient& client_;
  uint8_t machineAddr_;
  //FastCRC16 CRC16_;
  Crc16 CRC16_;
};

class GNFL : public JNGEdevice {
public:
  static const size_t RUN_SZ = 81;
  static const size_t BASIC_SZ = 87;

  explicit GNFL(uint8_t addr, WiFiClient& client);

  bool readBasicParams();
  bool readRunParams();

  bool runParamsIsSet() const { return runningParametersIsSet_; }
  bool basicParamsIsSet() const { return basicParametersIsSet_; }

  // buffers
  uint8_t runningParameters[RUN_SZ];
  uint8_t basicParameters[BASIC_SZ];

private:
  bool runningParametersIsSet_ = false;
  bool basicParametersIsSet_ = false;

  uint8_t cmdReadRunningParameters_[8] = {0xFF, 0x12, 0x10, 0x00, 0x00, 0x24, 0xA8, 0xCC};
  uint8_t cmdReadBasicParameters_[8]   = {0xFF, 0x16, 0x10, 0x24, 0x00, 0x27, 0x59, 0x06};
};