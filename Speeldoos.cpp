
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

#include "ssidinfo.h"
#include <EEPROM.h>
//MQTT
//#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>

#include <Wire.h>
//#include <Adafruit_ADS1015.h>
#include "ADS1X15.h"

#include "Adafruit_MCP23017.h"

//OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define SDA_PIN 4
#define SCL_PIN 5

#define ADS_FILTER 0.1

#define RANDOMLOOPTIME 2000 //2s
#define RANDOMLOOPLEDSPEED 50
#define GAMETIMEWAITFORBUTTON 30000 //30s
#define IDLETIMEOUT 120000 //2min
#define IDLEDELAY 3000 //3s
#define UPDATESPEED 100 //100ms

//program state
#define GAMEPUSHME    1
#define OFFSTATE      2

//Game Push Me
#define PUSHMESTART         1
#define PUSHMERANDOMLIGHTS  2
#define PUSHMEWAIT          3

//Random Lights
#define RANDOMLIGHTSSTART 1
#define RANDOMLIGHTSBLINK 2

//Send debug messages via MQTT
#define DEBUG 1

//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
ADS1115 ADS(0x48);
Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp4;

int programState;
int pushMeState;
int randomlightState;

const char* ssid = SSID_NAME;
const char* password = SSID_PASS;
const char* mqtt_server = MQTT_SERVER;

const char* mqtt_potmeter_0 = "/speeldoos/ads1"; //ADS1115 0
const char* mqtt_potmeter_1 = "/speeldoos/ads2"; //ADS1115 1
const char* mqtt_GPIO_0 = "/speeldoos/gpio1"; //MCP buttons
const char* mqtt_GPIO_1 = "/speeldoos/gpio2"; //MCP LED
const char* mqtt_debug = "/speeldoos/debug"; //MCP debug info
const char* mqtt_reaction_time = "/speeldoos/pushmereactiontime";
const char* mqtt_button_prefix = "/speeldoos/button/";
const char* mqtt_led_prefix = "/speeldoos/led/";

WiFiClient espClient;
PubSubClient client(espClient);
char mqttMsg[250];
char debugmsg[250];

int16_t mcp0GPIO, mcp0GPIO_old, mcp4GPIO, mcp4GPIO_old = 0;
int16_t adsresult0, adsresult0_old, adsresult1, adsresult1_old = 0;

byte target;
byte target_button_status;
unsigned long gameStartTime = 0;
unsigned long lastButtonPressTime = 0;
unsigned long randomLightsStartTime = 0;
unsigned long randomLightLastChangeTime = 0;
unsigned long lastUpdateSentTime = 0;

// Look Up Table  {button,led} //vibra motor pin A5=11
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

void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
int randomLights();
void mqttDebug(String message);
void sendMQTTUpdates();
int updateInputStatus();
void gamePushMe();
void goDeepSleep();
                 
void setupWifi() {

  delay(10);
  // We start by connecting to a WiFi network
  //mqttDebug(ssid);

  WiFi.mode(WIFI_STA);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());

  sprintf(debugmsg, "WiFi Connected to: %s", ssid); mqttDebug(debugmsg);
  //sprintf(debugmsg, "IP Address: %s", WiFi.localIP().toString()); mqttDebug(debugmsg);
  //OTA
  
  //WiFi.begin(ssid, password);
//  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    mqttDebug("Connection Failed! Rebooting...");
//    delay(5000);
//    ESP.restart();
//  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("speeldoos");

  ArduinoOTA.onStart([]() {
    mqttDebug("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    mqttDebug("OTA end, restarting...");
    ESP.restart(); //test if this works..
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    sprintf(debugmsg, "OTA progress: %u", (progress / (total / 100))); mqttDebug(debugmsg);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    sprintf(debugmsg, "OTA Error[%u]: ", error); mqttDebug(debugmsg);
    if (error == OTA_AUTH_ERROR) mqttDebug("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) mqttDebug("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) mqttDebug("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) mqttDebug("Receive Failed");
    else if (error == OTA_END_ERROR) mqttDebug("End Failed");
  });
  ArduinoOTA.begin();
  mqttDebug("OTA Ready");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  ////Serial.print("IP address: ");
  //mqttDebug(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    ////Serial.print((char)payload[i]);
  }

  // Switch on the LED if an 1 was received as first character
  //if ((char)payload[0] == '1') {
   
  //}

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    ////Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      mqttDebug("MQTT connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      ////Serial.print("failed, rc=");
      ////Serial.print(client.state());
      mqttDebug(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Debugging over MQTT
void mqttDebug(String message){
  if (DEBUG){
    char msg[250];
    message.toCharArray(msg,250);
    //publish sensor data to MQTT broker
    client.publish(mqtt_debug, msg);
  }
}

//show random lights and return 1 if done
int randomLights() {
  // run through random lights with certain delay
  switch (randomlightState) {
  case RANDOMLIGHTSSTART:
    randomLightsStartTime = millis();
    randomLightLastChangeTime = randomLightsStartTime;
    randomlightState = RANDOMLIGHTSBLINK;
    break;
  case RANDOMLIGHTSBLINK:
    if (millis()-randomLightLastChangeTime >= RANDOMLOOPLEDSPEED){ //Change LED when time is passed
      target = random(10);
      mcp4GPIO = 1<<lut[target][1]; //turn on only corresponding light.
      randomLightLastChangeTime = millis();
    }
    break;
  default:
    break;
  }
  
  // Check if we need to continue
  if (millis()-randomLightsStartTime >= RANDOMLOOPTIME){
    randomlightState = RANDOMLIGHTSSTART; //next time start
    return 1;
  }
  else {
    return 0;
  }
}
             

void sendMQTTUpdates(){
  // Send MQTT statuses
  String topic;
  
  if (mcp0GPIO != mcp0GPIO_old){  
    //Send On/Off to specific button topic.
    for (int i = 0; i < sizeof(lut)/2; i++) {
      String topic_prefix = mqtt_button_prefix; //two step string build needed
      topic = topic_prefix + i;
      if ((mcp0GPIO^mcp0GPIO_old) & 1<<lut[i][0]){ //Check if specific button is changed
        byte button_status = (mcp0GPIO&(1<<lut[i][0])) >> lut[i][0];
        if (button_status == LOW) client.publish(topic.c_str(), "ON");
        else client.publish(topic.c_str(), "OFF");   
      }                
    }
  }
  
  if (mcp4GPIO != mcp4GPIO_old){
    snprintf (mqttMsg, 250, "%d", mcp4GPIO);
    client.publish(mqtt_GPIO_1, mqttMsg);
  }

  if (millis() - lastUpdateSentTime >= UPDATESPEED){
    if (adsresult0 != adsresult0_old){
      snprintf (mqttMsg, 250, "%d", adsresult0);
      client.publish(mqtt_potmeter_0, mqttMsg);
    }
    if (adsresult1 != adsresult1_old){
      snprintf (mqttMsg, 250, "%d", adsresult1);
      client.publish(mqtt_potmeter_1, mqttMsg);
    }
    lastUpdateSentTime = millis();
  }
  
}

//Read and write User interface and send updates over MQTT
int updateInputStatus(){
  // Read Buttons
  mcp0GPIO_old = mcp0GPIO;
  mcp0GPIO = mcp0.readGPIOAB(); // read button status
  mcp4.writeGPIOAB(mcp4GPIO);   // write to LEDs
  //mcp4GPIO = mcp4.readGPIOAB(); // I believe they are the same...

  // ADS1115 draaiknopjes, connected op ADS0 en ADS1 (single ended)
  adsresult0_old = adsresult0;
  adsresult1_old = adsresult1;
  adsresult0 = int( (adsresult0_old*(1-ADS_FILTER)) + (ADS.readADC(0)*ADS_FILTER) ); //Some sort of filtering
  adsresult1 = int( (adsresult1_old*(1-ADS_FILTER)) + (ADS.readADC(1)*ADS_FILTER) );

  //if a button has been pressed, reset timeout timer
  if(programState == OFFSTATE){
    String text;
    mqttDebug("UPDATEINPUT: Recovering buttons for OFFSTATE");
    mcp0GPIO_old = EEPROM.read(0)<<8;
    mcp0GPIO_old = mcp0GPIO_old+EEPROM.read(1);
    text = "UPDATEINPUT: eepromMCP: "+String(mcp0GPIO_old, BIN);
    mqttDebug(text);
  }

  if (mcp0GPIO != mcp0GPIO_old){
    lastButtonPressTime = millis();
    mqttDebug("UPDATEINPUT: button changed");
    return 1;
  }
  else {
    return 0; // no button presses
  }
}

void gamePushMe(){
  //while in game
  
  switch (pushMeState) {
  case PUSHMERANDOMLIGHTS:
    if (randomLights()) {
      pushMeState = PUSHMESTART; //loop is done and continue
      mqttDebug("PUSHMERANDOMLIGHTS: loop done go to PUSHMESTART");
    }
    break;
    
  case PUSHMESTART: //Start up new Game
    target = random(10); //randomize button
    gameStartTime = millis();
    mcp4GPIO = 1<<lut[target][1]; //turn on only corresponding LED
    mqttDebug("PUSHMESTART: begin");
    if ( (mcp0GPIO&(1<<lut[target][0])) >> lut[target][0] == HIGH){ //check current status and save opposite
      target_button_status = LOW;
    }
    else { 
      target_button_status = HIGH;
    }
    
    //Start waiting game
    pushMeState = PUSHMEWAIT; //Go to next state
    mqttDebug("PUSHMESTART: go to PUSHMEWAIT");

    break;
    
  case PUSHMEWAIT: //Wait for button
    if ( millis() - gameStartTime <= GAMETIMEWAITFORBUTTON){
      if ((mcp0GPIO&(1<<lut[target][0])) >>lut[target][0] == target_button_status){
        // correct button set to desired position
        pushMeState = PUSHMERANDOMLIGHTS; //start new game
        //mqttDebug("PUSHMEWAIT: button pressed ");
        sprintf(debugmsg, "PUSHMEWAIT: button pressed after %u ms", (millis() - gameStartTime) ); mqttDebug(debugmsg);
        snprintf (mqttMsg, 50, "%u", (millis() - gameStartTime));
        client.publish(mqtt_reaction_time, mqttMsg);
      }
    }
    else {
      //timer passed
        pushMeState = PUSHMERANDOMLIGHTS; //start new game
        mqttDebug("PUSHMEWAIT: timer passed");
    }
    break;
  }
}

void setup() {
  //define the beginstates;
  programState = OFFSTATE;
  randomlightState = RANDOMLIGHTSSTART;
  pushMeState = PUSHMERANDOMLIGHTS;
  
  //Serial.begin(115200);
  EEPROM.begin(4);
  
  //SETUP Speeldoos
  Wire.begin(SDA_PIN,SCL_PIN);
  ADS.begin();        // only 1 ADS115
  mcp0.begin(0, &Wire);      // A0:A2 = Gnd
  mcp4.begin(4, &Wire);      // A0:A1 = Gnd A2 = VCC

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

void goDeepSleep(){
  bool eepromChanges = false;
  String text = "GODEEPSLEEP: ";
  mcp4GPIO = 0; // All LED off
  updateInputStatus();
  // Save button state for after deepsleep
  if ( (mcp0GPIO>>8) != EEPROM.read(0) ){
    EEPROM.write(0, (mcp0GPIO>>8));
    text = text + "eemprom1: "+String((mcp0GPIO>>8), BIN);
    eepromChanges = true;
  }
  if ( (byte)(mcp0GPIO&0x00FF) != EEPROM.read(1) ){
    EEPROM.write(1, (mcp0GPIO&0x00FF));
    text = text + " eeprom2: "+String((mcp0GPIO&0x00FF), BIN);
    eepromChanges = true;
  }
  if (eepromChanges){
    EEPROM.commit();
    mqttDebug(text);
  }
  mqttDebug("Last button press timeout, going deepsleep");
  delay(1000); // delay for updates?
  //programState = OFFSTATE;
  ESP.deepSleep(10*1000000, WAKE_RF_DEFAULT);
}

void loop() {
  int buttonsChanged;
  buttonsChanged = updateInputStatus(); //Read and write once per loop
  
  //Only use wifi when not in OFFSTATE
  if (programState != OFFSTATE){
     // OTA handles
    ArduinoOTA.handle();
    if (!client.connected()) {
      setupWifi();
      reconnect();
    }
    client.loop();
    sendMQTTUpdates();
  }
  
  mcp4GPIO_old = mcp4GPIO; // update old value because we update LEDs in software, but we use in mqtt sending
  
  switch (programState) {
  case GAMEPUSHME:
    // statements
    gamePushMe();
    break;
  case OFFSTATE:
    
    //wifi_set_sleep_type(MODEM_SLEEP_T) // Modem off
    if (buttonsChanged) {
      mqttDebug("OFFSTATE: buttonschanged");
      pushMeState = PUSHMERANDOMLIGHTS; //start new game with lights
      programState = GAMEPUSHME;
    }
    else {
      //mqttDebug("OFFSTATE: delay start");
      mqttDebug("OFFSTATE: going to DeepSleep 10s");
      ESP.deepSleep(10*1000000, WAKE_RF_DEFAULT);
      //delay(IDLEDELAY);
      //mqttDebug("OFFSTATE: delay end");
    }
    break;
  default:
    // statements
    break;
  }
  
  // timeout timer
  if ( (millis() - lastButtonPressTime) >= IDLETIMEOUT  && programState != OFFSTATE){
    goDeepSleep();
  }
  
}
