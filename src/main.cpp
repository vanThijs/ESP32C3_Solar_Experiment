//Experiment with an ESP32C3, combined with a small solar panel (6V panel) and a Lipo Battery. 
//Contents: 
//  Measures the Solar voltage (10k-20k VoltageDivider)
//  Battery voltage (10k-10k voltage divivder)
//  Combined with a water level sensor (Touch = Water present!) 
//  Data is being sent via MQTT

//ToDo: 
//  * Test power consumption
//  * Test deep sleep peripherals
//  * Test is INPUT_PULLUP is still active in deep sleep


#include <Arduino.h>
#include "secrets.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

//WiFi and MQTT
WiFiClient espClient;
PubSubClient PS_client(espClient);
unsigned long lastMsg = 0;


// Pin deinitions
const int WaterPin = 6;
const int BattPin_Analog = A2;
const int PanelPin_Analog = A3;

//Function prototypes
void setup_wifi();
float readPanelVoltage();
float readBatteryVoltage();
void reconnectMQTT();
void callback(char*, byte*, unsigned int);
void setup_wifi();
bool WaterLevelHigh();
void deep_sleep();
void SetLED(int,int,int); 

//Setup the NeoPixel
Adafruit_NeoPixel LED = Adafruit_NeoPixel(1, 7, NEO_RGB + NEO_KHZ800);

void setup() {
  Serial.begin(460800);
  LED.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  LED.clear(); // Set all pixel colors to 'off'
  LED.show();            // Turn OFF all pixels ASAP
  
  LED.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  //delay(7500);    //Delay to allow for serial monitor to connect 
  
  Serial.begin(460800);
  Serial.println("Starting up...\n");

  //Set the ADC resolution to 12 bits
  analogReadResolution(12);

  //Configure the deep sleep
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_s * 1000000);    //Sleep duration in seconds

  
  SetLED(0,0,127);
  setup_wifi();

  SetLED(0,127,0);
  PS_client.setServer(Broker, Port);
  PS_client.setCallback(callback);

  pinMode(WaterPin, INPUT_PULLUP);
  SetLED(0,0,0);
}


void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    //LED Green
    SetLED(0,127,0);

    //Read Sensor data
    float BatteryVoltage = readBatteryVoltage();
    float PanelVoltage = readPanelVoltage();
    bool WaterLevel = WaterLevelHigh();

    //Convert the values to char arrays
    char battString[8];
    char panelString[8];
    char waterString[12];

    if (WaterLevel == true) {
      strcpy(waterString, "Water High");
    } 
    else {
      strcpy(waterString, "Water OK");
    }
    dtostrf(BatteryVoltage, 1, 2, battString);
    dtostrf(PanelVoltage, 1, 2, panelString);

    //Print
    Serial.printf("Battery Voltage: %sV.\n", battString);
    Serial.printf("Panel Voltage: %sV.\n", panelString);
    Serial.printf("Water Level: %s.\n", waterString);

    if (!PS_client.connected()) {
      reconnectMQTT();
    }
    PS_client.loop();


    //Upload
    PS_client.publish(BAT_Topic, battString);
    PS_client.publish(PANEL_Topic, panelString);
    PS_client.publish(Level_Topic, waterString);

    //LED off
    SetLED(0,0,0);

    //Disconnect
    delay(500);
    WiFi.disconnect();
    
    //Disable RF hardware before sleep
    WiFi.mode(WIFI_OFF);

    //Disable GPIO6 (WaterSensor Pin)
    digitalWrite(WaterPin, LOW);    //Unsure if the pullup is still active in deep sleep, to be tested!

    //Deep sleep (30s)
    deep_sleep();
    //Serial.println("Should be sleeping now, but disabled for testing.");
  }
}




//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
// Read battery voltage ()Voltage divider 10k-10k)
float readBatteryVoltage() {
  int sensorValue = analogRead(BattPin_Analog);
  float BatteryVoltage = 2 * sensorValue * (3.3 / 4096.0);    // 2x voltage divider
  return BatteryVoltage;
}

// Read Solar panel voltage (Voltage divider 10k-22k)
float readPanelVoltage() {
  int sensorValue = analogRead(PanelPin_Analog);
  float PanelVoltage = 3.2 * sensorValue * (3.3 / 4096.0);    // Convert the voltage divided value back (10k-22k)
  return PanelVoltage;
}

//Read waterlvlPin, returns true if water is present
bool WaterLevelHigh() {
  bool WaterLevel = !digitalRead(WaterPin);
  return WaterLevel;
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
//Helper functions

//Deep sleep function
void deep_sleep() {
  //Serial.println("\nGoing to sleep now.\n\n\n");
  delay(10);
  esp_deep_sleep_start();
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.printf("\nConnecting to %S ", SSID);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//MQTT Callback function
void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;

  Serial.printf("Message arrived on topic: \"%S\": \"", topic);
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println("\".");
}

void reconnectMQTT() {
  while (!PS_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (PS_client.connect("ESP32_C3(Th)")) {
      Serial.println("connected");
      // Subscribe
      PS_client.subscribe(Sub_Topic);
    } 
    
    else {
      Serial.print("failed, rc=");
      Serial.print(PS_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void SetLED(int R, int G, int B) {
  LED.setPixelColor(0, LED.Color(R, G, B));
  LED.show();
}
