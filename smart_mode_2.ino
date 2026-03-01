#include <ESP8266WiFi.h>
#include "config.h"
#include "state.h"
#include "blinker.h"
#include "tcp_link.h"
#include "jnge.h"

#define LED_BUILTIN 2

enum class Mode : uint8_t { ACLOST, NIGHT, DAY };

State st;
WiFiClient tcpClient;
TcpLink link(tcpClient, IPAddress(10,10,100,254), TCP_PORT);
Blinker led(LED_BUILTIN, true);
GNFL jnge(MACHINE_ADDR, tcpClient);



//commands
unsigned char cmdN10[] = {0x06, 0x18, 0x10, 0x38, 0x00, 0x00, 0x00, 0x00}; //N10 inverter OFF voltage
unsigned char cmdN09[] = {0x06, 0x18, 0x10, 0x37, 0x00, 0x00, 0x00, 0x00}; //N09 inverter ON voltage


static void scheduleStatusBlink(Blinker& led, Mode m) {
  // не перебиваємо активний патерн
  if (led.active()) return;

  switch (m) {
    case Mode::ACLOST: led.start(8, 80, 150);  break;
    case Mode::NIGHT:  led.start(3, 50, 350);   break;
    case Mode::DAY:    led.start(3, 350, 50);     break;
  }
}

bool pollDeviceOnce() {
  bool ok1 = jnge.readRunParams();
  bool ok2 = jnge.readBasicParams();

  if (ok1 && ok2) return true;

  tcpClient.stop();
  return false;
}

void readAll(State& s) {
  s.batV = jnge.getParamF(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1006, 0.1, "Battery voltage");
  s.mainsV = jnge.getParamF(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1001, 0.1, "Municipal electric voltage");

  s.invOffRaw = jnge.getParamRaw(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1038);
  s.invOnRaw  = jnge.getParamRaw(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1037);

  s.bypassUv = jnge.getParamF(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1033, 0.1, "Bypass UV");
  s.bypassOv = jnge.getParamF(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1035, 0.1, "Bypass OV");

  s.activePowerRaw = jnge.getParamRaw(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1010);

  s.pvV = jnge.getParamF(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1020, 0.1, "PV voltage");
  s.pvP = jnge.getParamF(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1023, 0.1, "PV power");
}


// --- 1) decide state flags/counters (NO IO here) ---
void updateModes(State& s) {
  // detect AC lost/ok
  const bool acBad = (s.bypassUv > s.mainsV || s.mainsV > s.bypassOv);
  s.acIsLost = acBad;

  if (acBad) {
    s.acOkStreak = 0;
    if (!s.acLostMode) s.acDelay++;      // вхід в acLost
  } else {
    if (s.acOkStreak < 255) s.acOkStreak++;
    s.acDelay = 0;
  }

  // якщо вже acLostMode — day/night не рахуємо
  if (s.acLostMode) {
    s.dayDelay = 0;
    s.nightDelay = 0;
    return;
  }

  // Day preparing
  if (s.pvP >= MIN_PV_POWER || s.pvV >= MIN_PV_VOLTAGE) {
    if (s.nightMode && !s.acIsLost) {
      s.dayDelay++;
      s.nightDelay = 0;
      Serial.println("Try to enter day mode.");
    } else {
      Serial.println("The Day.");
      s.nightDelay = 0;
      s.acOkStreak = 0;
    }
  }

  // Night preparing
  if (s.pvP < MIN_PV_POWER && s.batV <= MIN_BAT_VOLT) {
    if (!s.nightMode && !s.acIsLost) {
      s.nightDelay++;
      s.dayDelay = 0;
      Serial.println("Try to enter night mode.");
    } else {
      Serial.println("The Night.");
      s.acOkStreak = 0;
      s.dayDelay = 0;
    }
  }
}

// --- 2) apply mode changes (IO: sendCommand) ---
void applyModes(State& s) {
  // enter acLost
  if (!s.acLostMode && s.acDelay >= AC_LOST_DELAY) {
      s.acLostMode = true;
      s.acDelay = 0;
      s.acOkStreak = 0;
      Serial.println("-----Enter AC Lost!-----");
    return; // важливо: після входу не робимо day/night в цьому циклі
  }

  // exit acLost (AC стабільно нормальний N циклів)
  if (s.acLostMode && !s.acIsLost && s.acOkStreak >= 3) {
    s.acLostMode = false;
    s.acOkStreak = 0;
    s.acDelay = 0;
    // після виходу НЕ треба відразу шити day/night тут.
    // нехай updateModes почне рахувати dayDelay/nightDelay з нуля.
    Serial.println("-----Exit AC Lost!-----");
    return;
  }

  if (s.acLostMode) return; // поки в acLost — не ліземо в day/night

  // Enter Day mode
  if (s.nightMode && s.dayDelay >= START_DAY_DELAY) {
      s.nightMode = false;
      s.dayDelay = 0;
      s.nightDelay = 0;
      Serial.println("-----Day mode is set!-----");
  }

  // Enter Night mode
  if (!s.nightMode && s.nightDelay >= START_NIGHT_DELAY) {
      s.nightMode = true;
      s.dayDelay = 0;
      s.nightDelay = 0;
      Serial.println("-----Night mode is set!-----");
  }
}

// --- 3) verify and self-heal inverter settings (IO: may sendCommand) ---
static inline Mode currentMode(const State& s) {
  if (s.acLostMode) return Mode::ACLOST;
  if (s.nightMode)  return Mode::NIGHT;
  return Mode::DAY;
}

static inline void voltagesForMode(Mode m, const State& s, uint16_t& onV, uint16_t& offV) {
  switch (m) {
    case Mode::ACLOST: {
      onV = (s.batV - 0.5) * 10;  offV = V_ACLOST_OFF;  //battery  depend voltage
      break;
    }
    case Mode::NIGHT:  onV = V_NIGHT_ON;   offV = V_NIGHT_OFF;   break;
    case Mode::DAY:    onV = V_DAY_ON;     offV = V_DAY_OFF;     break;
  }
}

void verifySettings(State& s) {
  uint16_t wantOn=0, wantOff=0;
  voltagesForMode(currentMode(s), s, wantOn, wantOff);
  Serial.printf("nightMode=%d acLostMode=%d want(off/on)=%u/%u inv(off/on)=%u/%u\n",
  st.nightMode, st.acLostMode, wantOff, wantOn, st.invOffRaw, st.invOnRaw);
  if (s.invOffRaw != wantOff) {
    Serial.println("Send to fix N10 (Off voltage)");
    jnge.sendCommand(cmdN10, sizeof(cmdN10), jnge.getMachineAddr(), wantOff);
  }
  if (s.invOnRaw != wantOn) {
    Serial.println("Send to fix N09 (On voltage)");
    jnge.sendCommand(cmdN09, sizeof(cmdN09), jnge.getMachineAddr(), wantOn);
  }
}

void printState(const State& s) {
  Serial.println(F("----------- STATE -----------"));

  Serial.print(F("Mode: "));
  if (s.acLostMode)      Serial.println(F("AC_LOST"));
  else if (s.nightMode)  Serial.println(F("NIGHT"));
  else                   Serial.println(F("DAY"));

  Serial.print(F("Delays -> dayMode: "));
  Serial.print(s.dayDelay);
  Serial.print(F(" | nightMode: "));
  Serial.print(s.nightDelay);
  Serial.print(F(" | acLostMode: "));
  Serial.println(s.acDelay);

  Serial.print(F("Flags -> acLostMode: "));
  Serial.print(s.acLostMode);
  Serial.print(F(" | acIsLost: "));
  Serial.print(s.acIsLost);
  Serial.print(F(" | acOkStreak: "));
  Serial.println(s.acOkStreak);

  Serial.print(F("Battery: "));
  Serial.print(s.batV, 2);
  Serial.print(F("V | PV: "));
  Serial.print(s.pvV, 2);
  Serial.print(F("V / "));
  Serial.print(s.pvP, 1);
  Serial.print(F("W | Mains: "));
  Serial.print(s.mainsV, 2);
  Serial.println(F("V"));

  Serial.print(F("InvVoltageRaw OFF/ON: "));
  Serial.print(s.invOffRaw);
  Serial.print(F(" / "));
  Serial.println(s.invOnRaw);

  Serial.println(F("-----------------------------"));
}

//-------------------------------SETUP--------------------------
void setup() {
  Serial.begin(9600);
  delay(300);

  led.begin();

  const uint32_t now = millis();
  st.lastPollMs  = now;   // старт таймера опитування
  st.lastBlinkMs = now;   // старт таймера блімка

  Serial.println();
  Serial.printf("Connecting to %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.print("\r\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // TCP best-effort до 5с
  uint32_t t0 = millis();
  while (!link.ensure(200) && (millis() - t0 < 5000UL)) {
    led.tick();
    delay(10);
  }

  if (!tcpClient.connected()) {
    Serial.println("TCP not connected yet; will retry in loop()");
    return;
  }

  Serial.println("Check current mode...");
  if (pollDeviceOnce()) {
    readAll(st);
    st.nightMode = (st.invOffRaw == V_NIGHT_OFF && st.invOnRaw == V_NIGHT_ON);
    Serial.println(st.nightMode ? "--Start in night mode" : "--Not in night mode");
  } else {
    Serial.println("Startup setup() failed");
  }
}

//-------------------------------LOOP----------------------------
void loop() {
  const uint32_t now = millis();

  led.tick();                     // завжди
  link.ensure(200);       // завжди

  // blink tick
  if (now - st.lastBlinkMs >= BLINK_EVERY_MS) {
    st.lastBlinkMs = now;
    scheduleStatusBlink(led, currentMode(st));
  }

  if (!tcpClient.connected()) return;

  // poll tick (стабільний)
  if (now - st.lastPollMs < LOOP_TIMER_MS) return;
  st.lastPollMs += LOOP_TIMER_MS;

  if (!pollDeviceOnce()) return;

  // далі: read -> compute -> act

  //printState(st); //print state
  readAll(st);        // зчитав, заповнив st.*
  updateModes(st);    // тікаються лічильники
  applyModes(st);     // змінили режим по лічильникам
  verifySettings(st); // переписати N09/N10, якщо поточні не відповідають режиму
  printState(st); //print state
}




//Full of the restart charging voltage  0x1040.
//Municipal electric charge rated current0x103F.

//Municipal electric charging state 0x1008 BIN inverse
/*
  0: Standby
  1: Constant charge
  2: Raise the charging
  3: Full of it
*/
//Inverter running state 0x100C BIN inverse
/*
  0: Standby 0
  1: Municipal electric charging soft start
  2: The inverter has a soft start
  3: Inverse runs normally
  4: Municipal power bypass
  5: Charging of municipal power bypass
  6: Failure mode
  7: Commissioning mode
*/
//Municipal electric charging current inverse? 0x1009 0.01

//PV Charging status 0x1022
/*
  00 Not being charged
  01 MPPT charging
  02 Boost charging
  03 Floating charging
  04 Balanced charging
*/

/*
   PV panel voltage 0x1020  33
   Total PV charging current 0x1022   34
  //Mode is Smart mode
  //OPEN invertor when sun is raise
  //PV charge > 150W
  06 18 10 37 00 83 D4 D0  //N09=13.1 default val
  06 18 10 38 00 76 24 94  //N10=11.8 discharge  limit at day
  After opening the machine will be in floating charge state near one hour
  then turn back to battery + PV


  //CLOSE invertor at night coming
  //PV charge < 150W
  06 18 10 38 00 82 25 13 //N09=15.0 can't reach this voltage
  06 18 10 37 00 96 15 1F //N10=13.0 discharge limit at night
  after night set the machine will raise E10 immediatly and
  will be with that error all night. Battery will be in floating charge
  Inverter status will be Bypass and charging
  C27=13.6 for lowering floating charge

  //mains AC lost condition
  06 18 10 37 00 83 D4 D0  //N09=13.1 default val
  06 18 10 38 00 6C A5 5F  //N10=10.8 discharge limit if cutoff the mains AC
*/
