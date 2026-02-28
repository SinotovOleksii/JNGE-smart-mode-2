#include "jnge.h"
#include <string.h>

JNGEdevice::JNGEdevice(uint8_t addr, WiFiClient& client)
  : client_(client), machineAddr_(addr) {
  sizeOfResponse = 0;
  memset(lastResponse, 0, sizeof(lastResponse));
}

bool JNGEdevice::readExact(uint8_t* dst, size_t need, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t t0 = millis();

  while (got < need) {
    if ((uint32_t)(millis() - t0) > timeoutMs) return false;

    int a = client_.available();
    if (a > 0) {
      size_t chunk = need - got;
      size_t r = client_.readBytes((char*)(dst + got), chunk);
      got += r;
    } else {
      delay(1); // yield for ESP8266
    }
  }
  return true;
}

bool JNGEdevice::readExactRetry(uint8_t* dst, size_t need, uint32_t timeoutMs, int tries) {
  for (int i = 0; i < tries; i++) {
    if (readExact(dst, need, timeoutMs)) return true;

    while (client_.available()) client_.read(); // flush
    delay(5);
  }
  return false;
}

size_t JNGEdevice::readUntilIdle(uint8_t* dst, size_t cap, uint32_t firstByteTimeoutMs, uint32_t idleGapMs) {
  uint32_t t0 = millis();
  while (client_.available() == 0) {
    if ((uint32_t)(millis() - t0) > firstByteTimeoutMs) return 0;
    delay(1);
  }

  size_t got = 0;
  uint32_t lastRx = millis();

  while (got < cap) {
    int a = client_.available();
    if (a > 0) {
      int b = client_.read();
      if (b < 0) break;
      dst[got++] = (uint8_t)b;
      lastRx = millis();
    } else {
      if ((uint32_t)(millis() - lastRx) >= idleGapMs) break;
      delay(1);
    }
  }
  return got;
}

bool JNGEdevice::sendCommand(uint8_t* cmd, size_t cmdSize, uint8_t addr, uint16_t value) {
  if (!client_.connected()) return false;
  if (cmdSize < 8) return false; // у тебе всі такі команди 8 байт

  // pack value + addr
  cmd[0] = addr;
  cmd[4] = (uint8_t)(value >> 8);
  cmd[5] = (uint8_t)(value & 0xFF);

  // crc -> buffer (LSB first)
  uint16_t crc = calcCRC16(cmd, (int)cmdSize - 2);
  cmd[6] = (uint8_t)(crc & 0xFF);
  cmd[7] = (uint8_t)(crc >> 8);

  // clear RX garbage
  while (client_.available()) client_.read();

  // send
  client_.write(cmd, cmdSize);

  //debug
  //Serial.println("send cmd");
  //printData(cmd, cmdSize);

  // read until idle
  uint8_t buf[MAX_LAST_RESPONSE];
  size_t got = readUntilIdle(buf, sizeof(buf), 2000, 100);

  if (got < 5) return false;
  if (got > MAX_LAST_RESPONSE) return false;

  if (got < 3) return false;
  if (!checkCRCresponse(buf, (int)got)) return false;

  if (buf[0] != addr) return false;

  sizeOfResponse = got;
  memcpy(lastResponse, buf, got);

  //debug
  //Serial.println("resp cmd");
  //printData(buf, got);
  return true;
}

uint16_t JNGEdevice::getParamRaw(const uint8_t* dataArr, int arrSize,
                                 uint16_t startAddr, uint16_t paramAddr,
                                 const char* caption) {
  int i1 = ((int)paramAddr - (int)startAddr) * 2 + 7;
  int i2 = i1 + 1;
  if (i2 >= arrSize || i1 < 0) {
    Serial.println("Error in param address.");
    return 0;
  }

  uint16_t wd = ((uint16_t)dataArr[i1] << 8) | dataArr[i2];

  if (caption && caption[0]) {
    Serial.print(caption);
    Serial.printf(",word:0x%X\n", wd);
  }
  return wd;
}

float JNGEdevice::getParamF(const uint8_t* dataArr, int arrSize,
                            uint16_t startAddr, uint16_t paramAddr,
                            float coefficient, const char* caption) {
  uint16_t wd = getParamRaw(dataArr, arrSize, startAddr, paramAddr, nullptr);
  float v = wd * coefficient;

  if (caption && caption[0]) {
    Serial.print(caption);
    Serial.printf(",word:0x%X,data:%4.2f\n", wd, v);
  }
  return v;
}

void JNGEdevice::printData(const uint8_t* dataArr, int arrSize) {
  Serial.print("\r\n-------print-data-------\r\n");
  for (int i = 0; i < arrSize; i++) {
    Serial.print(dataArr[i], HEX);
    Serial.print(((i + 1) % 16 == 0) ? "\r\n" : " ");
  }
  Serial.print("\r\n-------end--print-------\r\n");
}

uint16_t JNGEdevice::calcCRC16(uint8_t* dataArr, int arrSize) {
  CRC16_.clearCrc();
  //return CRC16_.modbus(dataArr, arrSize);
  return CRC16_.Modbus(dataArr, 0, arrSize);
}

bool JNGEdevice::checkCRCresponse(uint8_t* dataArr, int arrSize) {
  CRC16_.clearCrc();
  uint16_t crc = CRC16_.Modbus(dataArr, 0, arrSize - 2);
  uint8_t hi = crc >> 8;
  uint8_t lo = crc & 0xFF;

  if (lo == dataArr[arrSize - 2] && hi == dataArr[arrSize - 1]) return true;

  Serial.println("CRC check ERROR");
  return false;
}

// ---------------- GNFL ----------------

GNFL::GNFL(uint8_t addr, WiFiClient& client)
  : JNGEdevice(addr, client) {

  uint16_t crc;

  cmdReadRunningParameters_[0] = machineAddr_;
  crc = calcCRC16(cmdReadRunningParameters_, (int)sizeof(cmdReadRunningParameters_) - 2);
  cmdReadRunningParameters_[7] = (uint8_t)(crc >> 8);
  cmdReadRunningParameters_[6] = (uint8_t)(crc & 0xFF);

  cmdReadBasicParameters_[0] = machineAddr_;
  crc = calcCRC16(cmdReadBasicParameters_, (int)sizeof(cmdReadBasicParameters_) - 2);
  cmdReadBasicParameters_[7] = (uint8_t)(crc >> 8);
  cmdReadBasicParameters_[6] = (uint8_t)(crc & 0xFF);

  memset(runningParameters, 0, sizeof(runningParameters));
  memset(basicParameters, 0, sizeof(basicParameters));
}

bool GNFL::readBasicParams() {
  if (!client_.connected()) { basicParametersIsSet_ = false; return false; }

  while (client_.available()) client_.read();
  client_.write(cmdReadBasicParameters_, 8);

  //debug
  //Serial.println("req readBasicParams");
  //printData(cmdReadBasicParameters_, 8);

  if (!readExactRetry(basicParameters, sizeof(basicParameters), 2000, 2)) {
    basicParametersIsSet_ = false;
    return false;
  }
  //debug
  //Serial.println("resp readBasicParams");
  //printData(basicParameters, sizeof(basicParameters));

  if (basicParameters[0] != machineAddr_) { basicParametersIsSet_ = false; return false; }
  if (!checkCRCresponse(basicParameters, (int)sizeof(basicParameters))) { basicParametersIsSet_ = false; return false; }

  basicParametersIsSet_ = true;
  return true;
}

bool GNFL::readRunParams() {
  if (!client_.connected()) { runningParametersIsSet_ = false; return false; }

  while (client_.available()) client_.read();
  client_.write(cmdReadRunningParameters_, 8);
  //debug
  //Serial.println("req readRunParams");
  //printData(cmdReadRunningParameters_, 8);

  if (!readExactRetry(runningParameters, sizeof(runningParameters), 1200, 2)) {
    runningParametersIsSet_ = false;
    return false;
  }

  //debug
  //Serial.println("resp readRunParams");
  //printData(runningParameters, sizeof(runningParameters));

  if (runningParameters[0] != machineAddr_) { runningParametersIsSet_ = false; return false; }
  if (!checkCRCresponse(runningParameters, (int)sizeof(runningParameters))) { runningParametersIsSet_ = false; return false; }

  runningParametersIsSet_ = true;
  return true;
}