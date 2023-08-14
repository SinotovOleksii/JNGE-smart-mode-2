#include <FastCRC.h>
#include <ESP8266WiFi.h>
#define LED_BUILTIN 2

const char* ssid     = "EW11_A55A";
const char* password = "INeedConnect";
const uint16_t tcpPort = 8899;
unsigned char machineAddress = 0x06;
IPAddress server(10, 10, 100, 254);
WiFiClient tcpClient;

//system values
const int loopTimer = 60000; //ms, timer to start read the params
const int blinkTimer = 10000; //ms, timer to blink
unsigned long curMillis = millis(); //for loop pause

//device parameters
float currentInverterOffVoltage, currentInverterOnVoltage; //current values of voltage
float bypassUndervoltageProtection, bypassOvervoltageProtection; //current values of bypass protection
float activePower; // active output power of the inverter

//battery parameters
const int batteryString = 2; //day night mode battery voltage limits and mains AC lost condition
const int nightInverterOffVoltage = 130 * batteryString; //multiplied by 10 (0.1V)
const int nightInverterOnVoltage = 150 * batteryString;
const int dayInverterOffVoltage = 115 * batteryString;
const int dayInverterOnVoltage = 131 * batteryString;
const int lostAcInverterOffVoltage = 111 * batteryString;
const int lostAcInverterOnVoltage = 131 * batteryString;
const int lowPowerInverterOffVoltage = 124 * batteryString;


//variables for modes control
const float minPVpower = 150.0; //min power from PV
const float minPVvoltage = 88.0; //min voltage on PV array
const float minBatteryVoltage = 13.1 * batteryString; //min battery volatge to start count the night mode

//Day
const int startDayModeDelay = 10; //pauses to change mode

//Night
const int startNightModeDelay = 15; //pauses to change mode

//AC lost
const int AcLostModeDelay = 3; //pauses to change mode

//mode relating params
int currentNightModeDelay = 0; //counters for mode switching
int currentDayModeDelay = 0; //counters for mode switching
bool nightModeIsSet = false; //mode day/night
int currentAcLostModeDelay = 0; //counters for mode switching
bool mainsAcLostMode = false; //mains AC state
bool mainsAcIsLost = false; //AC realtime status need get from device




void blinker(int blinkCount, int delayON, int delayOFF){
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 1; i <= blinkCount; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayON);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayOFF);
  }
  
}

void checkClientConnection(WiFiClient* tcpClient) {
  while ( !tcpClient->connected() ) {
    Serial.print("Connecting to server ");
    Serial.println(server);
    if (!tcpClient->connect(server, tcpPort)) {
      Serial.println("Connection failed!");
      Serial.printf("Wifi status: %d", WiFi.status());
      Serial.print(" RRSI: ");
      Serial.println(WiFi.RSSI());
      delay(3000);
    } else { 
      Serial.println("Connected to server.");
      delay(1000);
    }
  }
  Serial.printf("Connected to the server: ");
  Serial.print(tcpClient->remoteIP());
  Serial.printf(":%d ", tcpClient->remotePort());
  Serial.printf("Wifi status: %d", WiFi.status());
  Serial.print(" RRSI: ");
  Serial.println(WiFi.RSSI());
}

class JNGEdevice {

  public:
    FastCRC16 CRC16;
    unsigned char* lastResponse;
    int sizeOfResponse;
    JNGEdevice(unsigned char addr, WiFiClient* clnt) {
      machineAddrr = addr;
      jngeClient = clnt;
      lastResponse = 0;
      sizeOfResponse = 0;
    };

    bool sendCommand(unsigned char* cmd, int cmdSize, uint8_t addr, uint16_t value) { //send cmd and save response addr=machine address, value=dword param
      if (jngeClient->connected()) {
        //transform value to bytes and set machine addr
        uint8_t b1 = value >> 8;
        uint8_t b2 = value & 0x00FF;
        cmd[4] = b1;
        cmd[5] = b2;
        cmd[0] = addr;
        //calc crc
        uint16_t crc = calcCRC16(cmd, cmdSize - 2);
        cmd[7] = crc >> 8;
        cmd[6] = crc & 0x00FF;
        if (!checkCRCresponse(cmd, cmdSize)){
          Serial.println("Abort sending the command. CRC error");
          return false;
        }
        Serial.print("Send command: ");
        printData(cmd, cmdSize);
        jngeClient->flush();
        while (jngeClient->available()) {
          jngeClient->read();
        }
        jngeClient->write(cmd, cmdSize);
        unsigned long timeout = millis(); //check exist of the response
        while (jngeClient->available() == 0) {
          if (millis() - timeout > 2000) {
            Serial.println("TCP Client Timeout !");
            jngeClient->stop();
            return false;
          }
        }
        int response = jngeClient->available();
        unsigned char arrResp[response];
        if (response > 0) {
          int sumNotZero = 0;
          for (int i = 0; i < response; i++) {
            arrResp[i] = jngeClient->read();
            sumNotZero += arrResp[i];
          }
          if (checkCRCresponse(arrResp, sizeof(arrResp)) && sumNotZero) {
            lastResponse = arrResp;
            sizeOfResponse = sizeof(arrResp);
            Serial.println("Response is: ");
            printData(arrResp, sizeof(arrResp));
            return true;
          } else {
            lastResponse = 0;
            sizeOfResponse = 0;
            return false;
          }
        }
      }
      return false;
    }

    float getParam(unsigned char* dataArr, int arrSize, uint16_t startAddr, uint16_t paramAddr, float coefficient, const char* caption) {
      uint8_t b1, b2;
      int addr1, addr2;
      addr1 = ((int)paramAddr - startAddr) * 2 + 7;
      addr2 = ((int)paramAddr - startAddr) * 2 + 8;
      if (addr1 >= arrSize || addr2 >= arrSize) {
        Serial.println("Error in param address.");
        return 0.0;
      }
      b1 = dataArr[addr1];
      b2 = dataArr[addr2];
      uint16_t wd = ((uint16_t)b1 << 8) | b2;
      if (caption != "") {
      Serial.print(caption);
      //Serial.printf(": b1:%X", b1);
      //Serial.printf(",b2:%X" , b2);     
      //Serial.printf(",addr:0x%X", paramAddr);
      Serial.printf(",world:0x%X", wd);
      Serial.printf(",data:%4.2f\n", wd * coefficient);
      }
      return wd * coefficient;
    }

    void printData(unsigned char *dataArr, int arrSize) {
      Serial.print("\r\n-------print-data-------\r\n");
      for (int i = 0; i < arrSize; i++) {
        Serial.print(dataArr[i], HEX);
        Serial.print(( i + 1 ) % 16 == 0 ? "\r\n" : " ");
      }
      Serial.print("\r\n-------end--print-------\r\n");
    }

    uint16_t calcCRC16(unsigned char *dataArr, int arrSize) {
      uint16_t crc = CRC16.modbus(dataArr, arrSize);
      //Serial.print("CRC calculated: 0x");
      //Serial.println(crc , HEX );
      return crc;
    }

    bool checkCRCresponse(unsigned char *dataArr, int arrSize) {
      uint16_t crc = CRC16.modbus(dataArr, arrSize - 2);
      uint8_t bite2 = crc >> 8;
      uint8_t bite1 = crc & 0x00FF;
      if (bite1 == dataArr[arrSize - 2] && bite2 == dataArr[arrSize - 1]) {
        //Serial.println("CRC check OK");
        return true;
      } else {
        Serial.println("CRC check ERROR");
        return false;
      }
    }

    unsigned char getMachineAddr(){
      return machineAddrr;
    }
    


  protected:
    WiFiClient* jngeClient;    
    unsigned char machineAddrr;
};

class GNFL : public JNGEdevice {
 public:
    unsigned char runningParameters[81];
    unsigned char basicParameters[87];
    
    GNFL(unsigned char addr, WiFiClient* clnt) : JNGEdevice(addr, clnt){
      //init commands and their crc
      uint16_t crc;
      cmdReadRunningParameters[0] = machineAddrr;
      crc = calcCRC16(cmdReadRunningParameters, sizeof(cmdReadRunningParameters) - 2);
      cmdReadRunningParameters[7] = crc >> 8;
      cmdReadRunningParameters[6] = crc & 0x00FF;
      cmdReadBasicParameters[0] = machineAddrr;
      crc = calcCRC16(cmdReadBasicParameters, sizeof(cmdReadBasicParameters) - 2);
      cmdReadBasicParameters[7] = crc >> 8;
      cmdReadBasicParameters[6] = crc & 0x00FF;
      //init error response
      
    }

    
    
    bool readBasicParams() { //read basic params and save it to obj
      int response;
      //Serial.print("readBasicParams.send cmd: ");
      if (jngeClient->connected()) {
        //printData(cmdReadBasicParameters, sizeof(cmdReadBasicParameters));
        jngeClient->flush();
        while (jngeClient->available()) {
          jngeClient->read();  //read all buffers before a cmd
        }
        jngeClient->write(cmdReadBasicParameters, 8);
        unsigned long timeout = millis(); //check exist of the response
        while (jngeClient->available() == 0) {
          if (millis() - timeout > 2000) {
            Serial.println("TCP Client Timeout !");
            jngeClient->stop();
            basicParametersIsSet = false;
            return false;
          }
        }
        response = jngeClient->available();
        if (response > 0 && response == sizeof(basicParameters)) {
          int sumNotZero = 0;
          for (int i = 0; i < response; i++) {
            basicParameters[i] = jngeClient->read();
            sumNotZero += basicParameters[i];
          }
          if (checkCRCresponse(basicParameters, sizeof(basicParameters)) && sumNotZero) {
            basicParametersIsSet = true;
            return true;
          } else {
            basicParametersIsSet = false;
            return false;
          }
        } else {
          Serial.print("Response error, resp size: ");
          Serial.println(response);
        }
      }
      basicParametersIsSet = false;
      return false;
    }

    bool readRunParams() { //read running params and save it to obj
      int response;
      //Serial.print("readRunParams.send cmd: ");
      if (jngeClient->connected()) {
        //printData(cmdReadRunningParameters, sizeof(cmdReadRunningParameters));
        jngeClient->flush();
        while (jngeClient->available()) {
          jngeClient->read();  //read all buffers before a cmd
        }
        jngeClient->write(cmdReadRunningParameters, 8);
        unsigned long timeout = millis(); //check exist of the response
        while (jngeClient->available() == 0) {
          if (millis() - timeout > 2000) {
            Serial.println("TCP Client Timeout !");
            jngeClient->stop();
            runningParametersIsSet = false;
            return false;
          }
        }
        response = jngeClient->available();
        if (response > 0 && response == sizeof(runningParameters)) {
          int sumNotZero = 0;
          for (int i = 0; i < response; i++) {
            runningParameters[i] = jngeClient->read();
            sumNotZero += runningParameters[i];
          }
          if (checkCRCresponse(runningParameters, sizeof(runningParameters)) && sumNotZero) {
            runningParametersIsSet = true;
            return true;
          } else {
            runningParametersIsSet = false;
            return false;
          }
        }  else {
          Serial.print("Response error, resp size: ");
          Serial.println(response);
        }
      }
      runningParametersIsSet = false;
      return false;
    }

    bool runParamsIsSet() {
      return runningParametersIsSet;
    }

    bool basicParamsIsSet() {
      return basicParametersIsSet;
    }

    private:
    bool runningParametersIsSet = false; //if response and crc is ok
    bool basicParametersIsSet = false; //if response and crc is ok
    unsigned char cmdReadRunningParameters[8] = {0xFF, 0x12, 0x10, 0x00, 0x00, 0x24, 0xA8, 0xCC};
    unsigned char cmdReadBasicParameters[8] = {0xFF, 0x16, 0x10, 0x24, 0x00, 0x27, 0x59, 0x06};
    unsigned char errorResponse[13] = {0xFF, 0xFF, 0x08, 0x24, 0x00, 0x27, 0x59, 0x06};
};

//the device
GNFL jnge(machineAddress, &tcpClient);

//commands
unsigned char cmdN10[] = {0x06, 0x18, 0x10, 0x38, 0x00, 0x00, 0x00, 0x00}; //N10 inverter OFF voltage
unsigned char cmdN09[] = {0x06, 0x18, 0x10, 0x37, 0x00, 0x00, 0x00, 0x00}; //N09 inverter ON voltage

void setup() {
  Serial.begin(9600);
  delay(300);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println();
  Serial.printf("Connecting to %s\n", ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(300);
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.print("\r\nWiFi connected.  IP address: ");
  Serial.println(WiFi.localIP());
  checkClientConnection(&tcpClient);
  delay(200);
  Serial.println("Check current mode");
  if (  jnge.readBasicParams() ) {
    int offVoltage = currentInverterOffVoltage = jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1038, 1, "");
    int onVoltage = currentInverterOnVoltage = jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1037, 1, "");
    if (offVoltage == nightInverterOffVoltage && onVoltage == nightInverterOnVoltage){
      Serial.println("Start in night mode");
      nightModeIsSet = true;
      blinker(5,100,200);
    } else {
      Serial.println("Not in night mode");
      blinker(2,100,300);
    }
  }
}



void loop() {
  unsigned long prevMillis = millis();
  if (prevMillis - curMillis < loopTimer) {
    //blinker
    if ((prevMillis - curMillis) % 10000 == 0){
      if (!mainsAcLostMode) {
        if (nightModeIsSet) {
            blinker(1,300,200);
            blinker(2,80,200);
        } else {
            blinker(2,80,200); 
            blinker(1,300,200);
        }
      } else {
        blinker(5,80,150);
      }
    }
    return;
  }
  
  checkClientConnection(&tcpClient);

  if (  jnge.readRunParams()  ) {
    //jnge.printData(jnge.runningParameters, sizeof(jnge.runningParameters));
    Serial.println("Running params was read");
  } 
  if (  jnge.readBasicParams() ) {
    //jnge.printData(jnge.basicParameters, sizeof(jnge.basicParameters));
    Serial.println("Basic params was read");
  }
  if (!jnge.runParamsIsSet() || !jnge.basicParamsIsSet()) return; 
  

  float batVoltage, PVpower, mainsVoltage, PVvoltage;
  float checkOffVoltage, checkOnVoltage;
  Serial.println("------------Battery's parameters------------------");
  batVoltage = jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1006, 0.1, "Battery voltage");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1014, 0.1, "Battery temperature");
  //jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1013, 1, "Battery temperature compensation voltage point");
  //jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x102E, 1, "Temperature compensation factor");
  Serial.println("------------Inverter and charger status------------");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x100C, 1, "Inverter running state");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1008, 1, "Municipal electric charging status");
  jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x103F, 0.01, "Municipal electric charge rated current");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x100D, 1, "Inverter internal state");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x101C, 1, "FAILURE CODE 1");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x101D, 1, "FAILURE CODE 2"); 
  //jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1017, 1, "Rated on the power");
  Serial.println("------------Settings----------------");
  jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1029, 0.1, "Floating charge voltage");
  jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1027, 0.1, "Boost charge voltage");
  //jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1040, 0.1, "Full of the restart charging voltage");
  currentInverterOffVoltage = jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1038, 1, "Inverse off voltage point N10");
  currentInverterOnVoltage = jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1037, 1, "Inverse open voltage point");
  bypassUndervoltageProtection = jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1033, 0.1, "Bypass voltage undervoltage protection point"); 
  bypassOvervoltageProtection = jnge.getParam(jnge.basicParameters, sizeof(jnge.basicParameters), 0x1024, 0x1035, 0.1, "Bypass voltage overvoltage protection point");
  Serial.println("------------Online power parameters----------------");
  activePower = jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1010, 1, "Active power");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1005, 0.01, "Current Inverse current");  
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1019, 0.1, "Voltage level");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1019, 0.01, "Current level");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1009, 0.01, "Municipal electric charging current");
  mainsVoltage = jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1001, 0.1, "Municipal electric voltage");

  Serial.println("------------PV power parameters----------------");
  //PV params
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1021, 0.1, "Total PV charging curren");
  PVvoltage = jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1020, 0.1, "PV panel voltage");
  jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1022, 1, "PV charging status");
  PVpower = jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1023, 0.1, "Photovoltaic charging power"); //real power
  //PVpower = jnge.getParam(jnge.runningParameters, sizeof(jnge.runningParameters), 0x1000, 0x1014, 0.1, "Photovoltaic charging power"); //power from temp sensor))

  
  //day mode preparing
  if (PVpower >= minPVpower || PVvoltage >= minPVvoltage) {
    if (nightModeIsSet && !mainsAcIsLost) { //try to enter in day mode
      currentDayModeDelay++;
      currentNightModeDelay = 0;
      Serial.println("Try to enter day mode.");
    } else { //else only clear counter of night mode
      Serial.println("The Day.");
      currentNightModeDelay = 0;
    }
  }
  //night mode preparing
  if (PVpower < minPVpower && batVoltage <= minBatteryVoltage) { 
    if (!nightModeIsSet && !mainsAcIsLost){ //try to enter in night mode
      currentNightModeDelay++;
      currentDayModeDelay = 0;
      Serial.println("Try to enter night mode.");
    } else { //else only clear counter of day mode
      Serial.println("The Night.");
      currentDayModeDelay = 0;
    }
  }
  //AC lost mode preparing
  if (bypassUndervoltageProtection > mainsVoltage || mainsVoltage > bypassOvervoltageProtection){ 
    Serial.println("AC is lost.");
    mainsAcIsLost = true;
    if (!mainsAcLostMode) {
      Serial.println("AC is lost. Try to enter mode");
      currentAcLostModeDelay++;  
    } 
  } else {
    Serial.println("AC in normal range.");
    mainsAcIsLost = false;
    currentAcLostModeDelay = 0;
    if (mainsAcLostMode) mainsAcLostMode = false;
  }

  //switch to AC lost mode it's working in parallel with smartmode 
  if (!mainsAcLostMode && currentAcLostModeDelay >= AcLostModeDelay) {
    Serial.println("AC Lost mode. try to program");  
    if (  jnge.sendCommand(cmdN10, sizeof(cmdN10), jnge.getMachineAddr(), lostAcInverterOffVoltage) && 
          jnge.sendCommand(cmdN09, sizeof(cmdN09), jnge.getMachineAddr(), lostAcInverterOnVoltage)  ) {  
      mainsAcLostMode = !mainsAcLostMode;
      currentAcLostModeDelay = 0;
      //night day mode
      Serial.println("-----AC lost mode is set!-----");
    }
  }
  //switch to day mode
  if (nightModeIsSet && currentDayModeDelay >= startDayModeDelay) {
    Serial.println("Day mode. try to program");
    if (  jnge.sendCommand(cmdN10, sizeof(cmdN10), jnge.getMachineAddr(), dayInverterOffVoltage) && 
          jnge.sendCommand(cmdN09, sizeof(cmdN09), jnge.getMachineAddr(), dayInverterOnVoltage)  ) {
      nightModeIsSet = !nightModeIsSet;
      currentDayModeDelay = 0;
      currentNightModeDelay = 0;
      Serial.println("-----Day mode is set!-----");
    }
  }
  //switch to night mode
  if (!nightModeIsSet && currentNightModeDelay >= startNightModeDelay) {
    Serial.println("Night mode. try to program");
    if (  jnge.sendCommand(cmdN10, sizeof(cmdN10), jnge.getMachineAddr(), nightInverterOffVoltage) && 
        jnge.sendCommand(cmdN09, sizeof(cmdN09), jnge.getMachineAddr(), nightInverterOnVoltage)  ) {
      nightModeIsSet = !nightModeIsSet;
      currentDayModeDelay = 0;
      currentNightModeDelay = 0;
      Serial.println("-----Night mode is set!-----");
    }
  }
  //check if parameters have written correctly
  if (nightModeIsSet && !mainsAcLostMode) {
    checkOnVoltage = nightInverterOnVoltage;
    checkOffVoltage = nightInverterOffVoltage;
  } 
  if (!nightModeIsSet && !mainsAcLostMode) {
    checkOnVoltage = dayInverterOnVoltage;
    checkOffVoltage = dayInverterOffVoltage;
  } 
  if (mainsAcLostMode) {
    checkOnVoltage = lostAcInverterOnVoltage;
    checkOffVoltage = lostAcInverterOffVoltage;
  }
  //check inverter settings
  if (checkOnVoltage && checkOffVoltage) {
    if (currentInverterOffVoltage == checkOffVoltage){
      Serial.println("The InverterOff parameter is correct.");
    } else { //try to set params
      Serial.println("The InverterOff parameter is incorrect.");
      jnge.sendCommand(cmdN10, sizeof(cmdN10), jnge.getMachineAddr(), checkOffVoltage);
    }
    if (currentInverterOnVoltage == checkOnVoltage){
      Serial.println("The InverterOn parameter is correct.");
    } else { //try to set params
      Serial.println("The InverterOn parameter is incorrect.");
      jnge.sendCommand(cmdN09, sizeof(cmdN09), jnge.getMachineAddr(), checkOnVoltage);
    }      
  }    
  Serial.printf("--- nightModeIsSet:%d ---", nightModeIsSet);
  Serial.printf(" currentNightModeDelay:%d ---", currentNightModeDelay);
  Serial.printf(" currentDayModeDelay:%d ---\n", currentDayModeDelay);
  Serial.printf("--- mainsAcLostMode:%d ---", mainsAcLostMode);
  Serial.printf(" currentAcLostModeDelay:%d ---\n", currentAcLostModeDelay);
  
  curMillis = millis(); //must be the latest sentance
};


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
