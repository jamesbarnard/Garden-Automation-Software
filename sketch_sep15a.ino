// ======== LIBRARIES ========
#include <SoftwareSerial.h>
// ===========================

// =============== PIN DEFINITIONS ===============
#define SW_EN        6  // Ensures pumps only on when turned on
#define RE_DE        7  // Receives LOW, Transmits HIGH
#define PUMP_WATER   8  // Pumps Pure Water
#define PUMP_pH_DECR  9 // Pumps pH Base
#define PUMP_pH_INCR 10 // Pumps pH Acid
#define PUMP_NITRO   11 // Pumps Nitro Solution
#define PUMP_PHOS    12 // Pumps Phos Solution
#define PUMP_POTA    13 // Pumps Pota Solution
// ===============================================

// ======= SOFTWARE SERIAL INITIALIZING =======
SoftwareSerial RS485(2,3);  // Rx/RO (Blue) & Tx/DI (Yellow)
// ============================================

// ==================== GLOBAL VARIABLE DECLARATIONS ====================
const byte reqFrame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
byte respFrame[19];
int bytesBack;
int rawConvValues[7];
int by;

float soilMoisture;
float soilTemperature;
float soilpH;
int soilNitro;
int soilPhos;
int soilPota;

int moistureLevel;
int pHLevel;
int nitroLevel;
int phosLevel;
int potaLevel;

long runTime;
long day = 86400000;
long hour = 3600000;
long minute = 60000;
long second = 1000;

// ======================================================================

// ================== SETUP CODE ==================
void setup() {
  // Setup Serial
  Serial.begin(4800);
  RS485.begin(4800);
  Serial.println("Starting Serial and RS485...");
  delay(200);

  // Setup Output Pins
  Serial.println("Setting up output pins...");
  pinMode(SW_EN, OUTPUT);
  pinMode(RE_DE, OUTPUT);
  pinMode(PUMP_WATER, OUTPUT);
  pinMode(PUMP_pH_DECR, OUTPUT);
  pinMode(PUMP_pH_INCR, OUTPUT);
  pinMode(PUMP_NITRO, OUTPUT);
  pinMode(PUMP_PHOS, OUTPUT);
  pinMode(PUMP_POTA, OUTPUT);
  delay(200);
  

  // Ensure all pumps off at setup
  setLow();
  
  Serial.println("Setup Complete");
  delay(600);
}
// ================================================

// ======== MICROCONTROLLER LOOP ========
void loop() {
  // Runtime
  runtime();
  
  // Reads the data from the sensor response
  sensorRead();

  // Converts the response to appropriate values
  hexConversion();
  
  // Displays the real values to Serial Monitor
  outputDisplay();

  // Test function for simulations
  testFunc();

  //Pump switching
  //digitalWrite(SW_EN, HIGH);
  //Check level
  //checkLevel();
  //Pumps on
  //sendDoses();
  //Run for 1 minute
  //delay(10000); // shorter while testing
  //Pumps off
  //setLow();

  //Serial.println("");
  delay(30000);  
}
// ============================================

// ======== RUNTIME ========
void runtime(){
  runTime = millis();
  int d = runTime / day;
  int h = (runTime % day) / hour;
  int m = ((runTime % day) % hour) / minute;
  int s = (((runTime % day) % hour) % minute) / second;

  /*Serial.print("Runtime: ");
  Serial.print(d);
  Serial.print("d ");
  Serial.print(h);
  Serial.print("h ");
  Serial.print(m);
  Serial.print("m ");
  Serial.print(s);
  Serial.println("s");*/

  // For test
  Serial.print(m);
  Serial.print(":");
  Serial.print(s);
}
// =========================

// ================ SENSOR READINGS ================
void sensorRead(){
  // Variables
  by = 0;
  
  // Timers
  unsigned long start = millis();
  const unsigned long timeout = 5000;

  // Set HIGH to send packet
  digitalWrite(RE_DE,HIGH);
  delay(10);

  // Check to see if the request is correct
  if (RS485.write(reqFrame, sizeof(reqFrame)) == 8){
    digitalWrite(RE_DE, LOW);

    // With a 19 byte response frame:
    while (by < 19){
      if (RS485.available()){
        respFrame[by] = RS485.read();
        by++;
      }
      // Timeout
      if (millis() - start >= timeout){
        Serial.println("Timeout!");
      }
    }
    /*
    Serial.println("++ New Reading ++\nResponse Frame:");
    for (byte i = 0; i < by; i++) {
      Serial.print(" | ");
      Serial.print(respFrame[i], HEX);
      delay(10);
    }
    int dataBytes = respFrame[2];
    Serial.println(" |");
    Serial.print("\t--> Bytes of Sensor Data Received: ");
    Serial.println(dataBytes);*/
  }
}
// =================================================

// ================ HEX TO DEC CONVERSION ================
void hexConversion() {
  // Variables
  int readValue;
  by = 3;

  for (int j = 0; j < 7; j++){
    readValue = respFrame[by] << 8 | respFrame[by+1];
    by = by + 2;
    rawConvValues[j] = readValue;
    delay(5);
  }

  // Prints raw converted values from sensor
  /*
  for (int k = 0; k < 7; k++){
    Serial.print(rawConvValues[k]);
    Serial.print(" / ");
  }
  Serial.println();
  */
}
// =======================================================

// ============ OUTPUT TO SERIAL MONITOR ============
void outputDisplay(){
  // Soil Moisture:
  soilMoisture = float(rawConvValues[0]) / 10;
  // Soil Temperature:
  soilTemperature = float(rawConvValues[1]) / 10;
  // Soil pH:
  soilpH = float(rawConvValues[3]) / 10;
  // Nitrogen:
  soilNitro = rawConvValues[4];
  // Phosphorous:
  soilPhos = rawConvValues[5];
  // Potassium:
  soilPota = rawConvValues[6];

  // Output:
  /*
  Serial.println();
  Serial.println("================================");
  Serial.print("1.  Soil Moisture:\t");
  Serial.print(soilMoisture,1);
  Serial.println("%");
  Serial.print("2.  Soil Temperature:\t");
  Serial.print(soilTemperature,1);
  Serial.println("°C");
  Serial.print("3.  Soil pH:\t\t");
  Serial.println(soilpH,1);
  Serial.print("4.  Soil Nitrogen:\t");
  Serial.print(soilNitro);
  Serial.println("mg/kg");
  Serial.print("5.  Soil Phosphorous:\t");
  Serial.print(soilPhos);
  Serial.println("mg/kg");
  Serial.print("6.  Soil Potassium:\t");
  Serial.print(soilPota);
  Serial.println("mg/kg");
  Serial.println("================================");*/
}
// =======================================================

// =========================================== CHECK LEVEL ===========================================
void checkLevel(){
  // Checks level in soil
  // running off a beans n-p-k -> 5-10-10
  float pHLow = 5.5;
  float pHHigh = 8.0;
  int idealN = 25;
  int idealP = 25;
  int idealK = 25;
  float idealMoist = 35.0;
  
  // pH Level:
  if (soilpH < pHLow){
    pHLevel = 1;
  }
  else if (soilpH > pHHigh){
    pHLevel = 2;
  }
  // Nitro Level:
  if (soilNitro < idealN){
    nitroLevel = 1;
  }
  else if (soilNitro > idealN){
    nitroLevel = 2;
  }
  // Phos Level:
  if (soilPhos < idealP){
    phosLevel = 1;
  }
  else if (soilPhos > idealP){
    phosLevel = 2;
  }
  // Pota Level:
  if (soilPota < idealK){
    potaLevel = 1;
  }
  else if (soilPota > idealK){
    potaLevel = 2;
  }
  // Moisture Content:
  // first check if any other solutions are being injected
  if ((pHLevel == 1) || (pHLevel == 2) || (nitroLevel == 1) || (phosLevel == 1) || (potaLevel == 1)){
    moistureLevel = 0;
  }
  // adds water if nutrient too high or water too low
  else if ((nitroLevel == 2) || (phosLevel == 2) || (potaLevel == 2) || (soilMoisture < idealMoist)){
    moistureLevel = 1;
  }
  
}
// ===================================================================================================

// =========== SET LOW ===========
void setLow(){
  // Set all pumps low
  Serial.println("Pumps OFF");
  digitalWrite(PUMP_WATER, LOW);
  digitalWrite(PUMP_pH_DECR, LOW);
  digitalWrite(PUMP_pH_INCR, LOW);
  digitalWrite(PUMP_NITRO, LOW);
  digitalWrite(PUMP_PHOS, LOW);
  digitalWrite(PUMP_POTA, LOW);
  
  // Reset all level checks
  moistureLevel = 0;
  pHLevel = 0;
  nitroLevel = 0;
  phosLevel = 0;
  potaLevel = 0;
}
//  ===============================

// =========== SEND DOSE ===========
void sendDoses(){
  // Set all necessary pumps high, run, turn off after dose
  // Pumps flow at 1.09L/min. A dose is 100mL = ~5.5s
  int dose = 5500; // In millisec
  Serial.print("Pumps ON: ");
  if (pHLevel == 1){
    digitalWrite(PUMP_pH_INCR, HIGH);
    Serial.print("pH-I . ");
    delay(dose);
    digitalWrite(PUMP_pH_INCR, LOW);
  }
  if (pHLevel == 2){
    digitalWrite(PUMP_pH_DECR, HIGH);
    Serial.print("pH-D . ");
    delay(dose);
    digitalWrite(PUMP_pH_DECR, LOW);
  }
  if (nitroLevel == 1){
    digitalWrite(PUMP_NITRO, HIGH);
    Serial.print("N . ");
    delay(dose);
    digitalWrite(PUMP_NITRO, LOW);
  }
  if (phosLevel == 1){
    digitalWrite(PUMP_PHOS, HIGH);
    Serial.print("P . ");
    delay(dose);
    digitalWrite(PUMP_PHOS, LOW);
  }
  if (potaLevel == 1){
    digitalWrite(PUMP_POTA, HIGH);
    Serial.print("K . ");
    delay(dose);
    digitalWrite(PUMP_POTA, LOW);
  }
  if (moistureLevel == 1){
   digitalWrite(PUMP_WATER, HIGH);
   Serial.print("Water . ");
   delay(dose);
   digitalWrite(PUMP_WATER, LOW);
  }
  Serial.println();
}
//  ===============================


// =================== TEST FUNCTION ===================

void testFunc(){
  //Serial.println("Next Entry:");
  Serial.print(soilMoisture,1);
  Serial.print("|");
  Serial.print(soilTemperature,1);
  Serial.print("|");
  Serial.print(soilpH,1);
  Serial.print("|");
  Serial.print(soilNitro);
  Serial.print("|");
  Serial.print(soilPhos);
  Serial.print("|");
  Serial.println(soilPota);
}

// =====================================================
