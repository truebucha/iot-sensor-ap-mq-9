/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>


// replace with your channel’s thingspeak API key,
// tutorial https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server

const char WiFiApName[] = "sensor-co-";
const char WiFiApSecret[] = "Sensor123";
const float SEC = 1e3;
WiFiServer server(8090);
Adafruit_BME280 bme; // I2C

/**************************
 *   S E T U P
 **************************/

 void setupWiFi() {
  byte mac[6];
  WiFi.macAddress(mac);

  Serial.print(F("MAC: "));
  Serial.print(mac[5],HEX);
  Serial.print(F(":"));
  Serial.print(mac[4],HEX);
  Serial.print(F(":"));
  Serial.print(mac[3],HEX);
  Serial.print(F(":"));
  Serial.print(mac[2],HEX);
  Serial.print(F(":"));
  Serial.print(mac[1],HEX);
  Serial.print(F(":"));
  Serial.println(mac[0],HEX);

  String apName = WiFiApName;
  apName += String(mac[5], HEX);
  apName += String(mac[4], HEX);

  Serial.print(F("ApName: "));
  Serial.println(apName);

  WiFi.mode(WIFI_AP);

  char ApName[15];
  apName.toCharArray(ApName, 15);

  WiFi.softAP(ApName, WiFiApSecret);
  Serial.println(F("Finished with wifi setup"));
}


 // void initBme() {
 //   Wire.begin(5,4);
 //   if (!bme.begin(0x76)) {
 //     Serial.println("Could not find a valid bme280 sensor, check wiring!");
 //     ESP.deepSleep( 30 * 1000000, WAKE_RF_DISABLED);
 //   } else {
 //     Serial.println(F("BME280 found!"));
 //   }
 //   Serial.println("Finished with BME setup");
 // }

void initHardware() {
   pinMode(2, OUTPUT);
   pinMode(5, OUTPUT);
   digitalWrite(5, LOW);
   //initBme();
   Serial.println("Finished with hardware setup");
   // Don't need to set ANALOG_PIN as input,
   // that's all it can be.
}

void setup() {
  Serial.begin(74880);
  initHardware();
  delay(2e3);
  Serial.println("Start board setup");

  digitalWrite(2, LOW);
  setupWiFi();
  server.begin();
  digitalWrite(2, HIGH);
  Serial.println("Finished with setup");
}

  /**************************
 *  L O O P
 **************************/

float miliVoltsUsingAnalogRead(int value) {
  int zeroVoltageValue = 5;
  int correctedValue = value - zeroVoltageValue;
  int input = correctedValue > 0 ? correctedValue : 0;
  // measured 3.12 v source stabilazed voltage
  //          Usource(mV) / stepsTotal * currentSteps
  float result = (3120.0 / float(1024.0-zeroVoltageValue) * float(input));
  // Serial.print("read :");
  // Serial.println(value);
  // Serial.println(result);
  return result;
}

float ppmUsingMiliVolts(float mV) {
  //          sensor U * (10e6(mA to nA) / current resistor(50kOm))
  float nA = mV * (1000000 / 50000);
  //                  sensor current  * sensor nA->ppm coefficient / amplifier coefficient * correction;
  float result = nA * 1.4 / 108 * 6.11;
  // Serial.print("ppm :");
  // Serial.println(nA);
  // Serial.println(result);
  return result;
}


int analog = 0;
float mV = 0.0;
float coPpm = 0.1;
float temp = 20.0;
float pressure = 97.8;
float humidity = 43.0;

void readBmeValues() {
  temp = bme.readTemperature();
  pressure = bme.readPressure();
  humidity = bme.readHumidity();
}

void readCoValues() {
  analog = analogRead(0);
  mV = miliVoltsUsingAnalogRead(analog);
  coPpm = ppmUsingMiliVolts(mV);
}

float s_volt() {
  int sensorValue = analogRead(A0);
  float result =(float)sensorValue/1024*3.3;
  Serial.print("\n");
  Serial.print("sensor_volt = ");
  Serial.println(result);
  Serial.print("\n");
  return result;
}

float processValue() {
  digitalWrite(5, HIGH);
  Serial.println("begin heating");
  s_volt();
  delay(60 * SEC);
  s_volt();
  Serial.println("begin cooling");
  digitalWrite(5, LOW);
  delay(90 * SEC);
  Serial.println("stop cooling");
  digitalWrite(5, HIGH);
  Serial.println("measure");
  float result=s_volt();
  digitalWrite(5, LOW);
  Serial.println("done");
  return result;
}

float measure(float RZero) {
  Serial.println("== measurement ==");
  float sensor_volt;
  float RS_gas; // Get value of RS in a GAS
  float ratio; // Get ratio RS_GAS/RS_air

  sensor_volt = processValue();
  RS_gas = (3.3-sensor_volt)/sensor_volt; // omit *RL

  /*-Replace the name "R0" with the value of R0 in the demo of First Test -*/
  ratio = RS_gas/RZero;  // ratio = RS/R0
  /*-----------------------------------------------------------------------*/
  Serial.print("R0 = ");
  Serial.println(RZero);
  Serial.print("sensor_volt = ");
  Serial.println(sensor_volt);
  Serial.print("RS_ratio = ");
  Serial.println(RS_gas);
  Serial.print("Rs/R0 = ");
  Serial.println(ratio);

  Serial.print("\n\n");
  return sensor_volt;
}

float calibrate() {
  Serial.println("== calibration ==");
  float sensor_volt;
    float RS_air; //  Get the value of RS via in a clear air
    float R0;  // Get the value of R0 via in LPG
    float sensorValue;

    processValue();
    digitalWrite(5, HIGH);
    Serial.println("measure average");
    /*--- Get a average data by testing 100 times ---*/

    for(int x = 0 ; x < 100 ; x++)
    {
      sensorValue = sensorValue + analogRead(A0);
    }
    sensorValue = sensorValue/100.0;
    digitalWrite(5, LOW);

    sensor_volt = sensorValue/1024*3.3;
    RS_air = (3.3-sensor_volt)/sensor_volt; // omit *RL
    R0 = RS_air/9.9; // The ratio of RS/R0 is 9.9 in LPG gas

    Serial.print("sensor_volt = ");
    Serial.print(sensor_volt);
    Serial.println("V");

    Serial.print("R0 = ");
    Serial.println(R0);
    Serial.println("done");
    Serial.print("\n\n");
    return R0;
}

void send(float coPpm) {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  digitalWrite(2, LOW);

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);

  client.flush();

  // if (req.indexOf("/led/0") != -1)

  // Prepare the response. Start with the common header:
//  readCoValues();
//  readBmeValues();
  // coPpm += coPpm / 5;

  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: application/json\r\n\r\n";
  s += "{";
  s += "\"analog\":";
  s += String(analog);
  // s += ",";
  //
  // s += "\"temp\":";
  // s += String(temp);
  // s += ",";
  //
  // s += "\"pressure\":";
  // s += String(pressure);
  // s += ",";
  //
  // s += "\"humidity\":";
  // s += String(humidity);

  s += "}";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");
  digitalWrite(2, HIGH);
  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}

float R_Zero = 0.0;
void loop() {
  Serial.println("loop");
  if (R_Zero == 0.0) {
    float R0 = 0.0;
    R0 = calibrate();
    yield();
    delay(30 * SEC);
    R_Zero = R0;
  }

  float result = measure(R_Zero);
  send(result);
  delay(1 * SEC);
}
