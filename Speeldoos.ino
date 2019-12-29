
/*
 *  Bottom Row buttons
 * Button7 = LED11
 * Button6 = LED12
 * Button5 = LED13
 * Button4 = LED14
 * Button3 = LED15
 * 
 *  Top micro buttons
 * Button2 = LED10
 * Button1 = LED9
 * 
 *  Switch single
 * Button8 = LED7
 * 
 *  Switch Double
 * Button9 = LED5
 * Button10 = LED5
 */


#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "Adafruit_MCP23017.h"

//OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define SDA_PIN 4
#define SCL_PIN 5

#define RANDOMLOOPTIME 2000 //2s
#define GAMETIMEWAITFORBUTTON 20000 //20s

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp4;

const char* ssid = "**;
const char* password = "**";

int16_t mcp0GPIO, mcp4GPIO;
int16_t adsresult0, adsresult1;

byte target;
byte target_button_status;
unsigned long gameStartTime;

// Look Up Table  {button,led}
byte lut[][2] = { {7,8}, //Button 1
                  {6,9}, //Button 2
                  {5,10}, //Button 3
                  {4,11}, //Button 4
                  {3,12}, //Button 5
                  {10,5}, //Switch right up
                  {9,5},  //Switch right down
                  {1,14}, //Micro button links
                  {2,13}, //Micro button rechts
                  {8,7} };  //Switch left
                 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("speeldoos");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

 //SETUP Speeldoos
  Wire.begin(SDA_PIN,SCL_PIN);
  ads.begin();        // only 1 ADS115
  mcp0.begin(0);      // A0:A2 = Gnd
  mcp4.begin(4);      // A0:A1 = Gnd A2 = VCC

 // MCP buttons left side
  mcp0.pinMode(7+8, INPUT);
  mcp0.pinMode(6+8, INPUT);
  mcp0.pinMode(5+8, INPUT);
  mcp0.pinMode(4+8, INPUT);
  mcp0.pinMode(3+8, INPUT);
  mcp0.pinMode(2+8, INPUT);
  mcp0.pinMode(1+8, INPUT);
  mcp0.pinMode(0+8, INPUT);
 // MCP buttons right side
  mcp0.pinMode(7, INPUT);
  mcp0.pinMode(6, INPUT);
  mcp0.pinMode(5, INPUT);
  mcp0.pinMode(4, INPUT);
  mcp0.pinMode(3, INPUT);
  mcp0.pinMode(2, INPUT);
  mcp0.pinMode(1, INPUT);
  mcp0.pinMode(0, INPUT);
  

 // MCP LED left side
  mcp4.pinMode(7+8, OUTPUT);
  mcp4.pinMode(6+8, OUTPUT);
  mcp4.pinMode(5+8, OUTPUT);
  mcp4.pinMode(4+8, OUTPUT);
  mcp4.pinMode(3+8, OUTPUT);
  mcp4.pinMode(2+8, OUTPUT);
  mcp4.pinMode(1+8, OUTPUT);
  mcp4.pinMode(0+8, OUTPUT);
 // MCP LED right side
  mcp4.pinMode(7, OUTPUT);
  mcp4.pinMode(6, OUTPUT);
  mcp4.pinMode(5, OUTPUT);
  mcp4.pinMode(4, OUTPUT);
  mcp4.pinMode(3, OUTPUT);
  mcp4.pinMode(2, OUTPUT);
  mcp4.pinMode(1, OUTPUT);
  mcp4.pinMode(0, OUTPUT);


/*
// Set all pins on mcp0 to input for buttons
  for (int i=0; 1<16; i++) {
    //mcp0.pinMode(i, INPUT); 
  }

// Set all pins on mcp4 to output for LED
  for (int i=0; 1<16; i++) {
   // mcp4.pinMode(i, OUTPUT); 
  }
*/
  //mcp4GPIO = 1; //reset LED
  
}

void randomLights()
{
  // run through random lights with certain delay
  long start = millis();

  while (millis()-start <= RANDOMLOOPTIME){
    target = random(10);
    delay(int(adsresult0/100));
    mcp4.writeGPIOAB(1<<lut[target][1]);
    //mcp4.digitalWrite(lut[target][1], HIGH);
  }
 
}

void startGame(){
  randomLights(); //run random lights to start
  target = random(10); //randomize button
  gameStartTime = millis();
  mcp4.writeGPIOAB(1<<lut[target][1]); //turn on only corresponding LED
  if ( mcp0.digitalRead(lut[target][0]) == HIGH){ //check current status and save opposite
    target_button_status = LOW;
  }
  else { 
    target_button_status = HIGH;
  }
  Serial.print("Target: "); Serial.print(target); Serial.print(" to: "); Serial.println(target_button_status);
}

void loop() {
  ArduinoOTA.handle();
  
  // put your main code here, to run repeatedly:
  //mcp0GPIO = mcp0.readGPIOAB();
  //mcp4GPIO = mcp4.readGPIOAB();
  
  //Serial.print("mcp0GPIO: "); Serial.println(mcp0GPIO);

// ADS1115 draaiknopjes, connected op ADS0 en ADS1 (single ended)
  adsresult0 = ads.readADC_SingleEnded(0);
  adsresult1 = ads.readADC_SingleEnded(1);
  //Serial.print("ads0: "); Serial.println(adsresult0);
  //Serial.print("ads1: "); Serial.println(adsresult1);
  
  
  //while in game:
  if ( millis() - gameStartTime <= GAMETIMEWAITFORBUTTON){
    if (mcp0.digitalRead(lut[target][0]) == target_button_status){
      // correct button set to desired position
      startGame(); //start new game
    }
  }
  else {
    //timer passed
    startGame();
  }
  
}
