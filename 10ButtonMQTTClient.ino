#include <WiFi.h>
#include <PubSubClient.h>
#include "InputDebounce.h"

#define BUTTON_DEBOUNCE_DELAY 20   // [ms]

static const int pinLED = LED_BUILTIN; // 13
static int pinPB[11];
int i;

// Network Setup Information

const char* ssid = "VFCFalloutTrackerA";
const char* wifi_password = "BuildTeam32550";
const char* mqtt_server = "192.168.6.1";  // IP of the MQTT broker (raspberry pi)
const char* mqtt_username = "VFCA";
const char* mqtt_password = "";
const char* clientID = "FalloutA_ESP32"; // MQTT client ID
const char* button_topic = "falloutdash/button";

WiFiClient wifiClient;
PubSubClient mqtt_client(mqtt_server, 1883, wifiClient); 

class MyInputDebounce : public InputDebounce
{
public:
  MyInputDebounce(int8_t pinIn = -1, unsigned long debDelay = DEFAULT_INPUT_DEBOUNCE_DELAY, PinInMode pinInMode = PIM_INT_PULL_UP_RES, unsigned long pressedDuration = 0)
    : InputDebounce(pinIn, debDelay, pinInMode, pressedDuration)
    , _pinLED(-1)
  {}
  virtual ~MyInputDebounce()
  {}
  void setPinLED(int8_t pinLED)
  {
    _pinLED = pinLED;
  }
protected:
  virtual void pressed()
  {
    // handle pressed state
    if(_pinLED >= 0) {
      digitalWrite(_pinLED, HIGH); // turn the LED on
    }
    Serial.print("HIGH (pin: ");
    Serial.print(getPinIn());
    Serial.println(")");
  }
  virtual void released()
  {
    // handle released state
    if(_pinLED >= 0) {
      digitalWrite(_pinLED, LOW); // turn the LED off
    }
    Serial.print("LOW (pin: ");
    Serial.print(getPinIn());
    Serial.println(")");
    mqtt_client.publish(button_topic,String(i).c_str());
  }
  virtual void pressedDuration(unsigned long duration)
  {
    // handle still pressed state
    Serial.print("HIGH (pin: ");
    Serial.print(getPinIn());
    Serial.print(") still pressed, duration ");
    Serial.print(duration);
    Serial.println("ms");
  }
  virtual void releasedDuration(unsigned long duration)
  {
    // handle released state
    Serial.print("LOW (pin: ");
    Serial.print(getPinIn());
    Serial.print("), duration ");
    Serial.print(duration);
    Serial.println("ms");
  }
private:
  int8_t _pinLED;
};

static MyInputDebounce PB[11];

void setup_wifi() {
    
    WiFi.begin(ssid, wifi_password);
    WiFi.setHostname("espTenButtonMQTT");

    Serial.print("Connecting to ");
    Serial.print(ssid);  

    int connectdelays = 0;

    while (WiFi.status() != WL_CONNECTED) {
        connectdelays++;
        delay(500);
        Serial.print(".");
        if (connectdelays > 10){break;}
    }

    if(WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected!");
    }
  
}

void mqtt_reconnect() {
    
    if(!mqtt_client.connected()){
        Serial.print("Attempting mqtt connection...");
        if (mqtt_client.connect(clientID, mqtt_username, mqtt_password)) {
            Serial.println("connected");
        }else{
            Serial.print("mqtt connection failed, rc=");
            Serial.println(mqtt_client.state());
            delay(500);
        }
    }
}

void setup(){
  
    // Set pins for each button number
    pinMode(pinLED, OUTPUT);
    pinPB[1] = 34;
    pinPB[2] = 39;
    pinPB[3] = 27;
    pinPB[4] = 33;
    pinPB[5] = 15;
    pinPB[6] = 32;
    pinPB[7] = 14;
    pinPB[8] = 36;
    pinPB[9] = 4;
    pinPB[10] = 21;
  
  // init serial
  Serial.begin(9600);
  
  // setup input buttons (debounced)
  for(int i = 1; i <= 10; i++){
      PB[i].setPinLED(pinLED);
      PB[i].setup(pinPB[i], BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);
  }
}

void loop(){
  
  if (WiFi.status() != WL_CONNECTED){
      setup_wifi();
  }

  if (WiFi.status() == WL_CONNECTED && !mqtt_client.connected()) {
      mqtt_reconnect();
  }
  
  mqtt_client.loop();

  // Record time at beginning of scan for debounce calculations
  unsigned long scanstart = millis();

  for(i = 1; i <= 10; i++){
      PB[i].process(scanstart);  
  }
    
  delay(1); // [ms]
}
