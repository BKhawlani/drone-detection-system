#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// ============================================================
// Hazirlama
// ============================================================
//const char* WIFI_SSID= "TUGVA-KASTAMONU";
//const char* WIFI_PASSWORD= "Tgv@2024!!";
const char* WIFI_SSID= "Project";
const char* WIFI_PASSWORD= "12345678";
const char* MQTT_BROKER = "192.168.43.38";
const int   MQTT_PORT   = 1883;
const char* TOPIC_COMMAND     = "drone/command";
const char* TOPIC_COORDINATES = "drone/coordinates";


#define RED_LED_PIN    26
#define GREEN_LED_PIN  27
#define BUZZER_PIN     25
#define SDA_PIN        21
#define SCL_PIN        22

// ============================================================
// Baglama nesneler tanimlama
// ============================================================
WiFiClient espClient;
PubSubClient mqttClient(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================
// degisken tanimlama
// ============================================================
bool droneDetected = false;
int droneX = 0, droneY = 0;
int droneWidth = 0, droneHeight = 0;
float droneConfidence = 0; //guven skoru

unsigned long totalDetections = 0; 
unsigned long coordinateUpdates = 0; 

// ============================================================
// BUZZER sesi zamanlama
// ============================================================
unsigned long lastBeepTime = 0;
bool buzzerState = false;
const unsigned long BEEP_INTERVAL = 200;
// ============================================================
// LCD zamanlama
// ============================================================
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 500;
int lcdPage = 0;


// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN); //lcd init
  lcd.init();
  lcd.backlight();



  lcd.setCursor(0,0);// konumlandirma 
  lcd.print("Drone Detection");
  lcd.setCursor(0,1);
  lcd.print("System Starting");

  connectWiFi();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  connectMQTT();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Status: READY");
  lcd.setCursor(0,1);
  lcd.print("Area: SAFE");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  handleBuzzer();
  updateLCD();
}

// ============================================================
// WIFI
// ============================================================
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
  }
  digitalWrite(GREEN_LED_PIN, HIGH);
}

// ============================================================
// MQTT
// ============================================================
void connectMQTT() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_DroneDetector")) {
      mqttClient.subscribe(TOPIC_COMMAND);
      mqttClient.subscribe(TOPIC_COORDINATES);
    } else {
      delay(2000);
    }
  }
}

// ============================================================
// CALLBACK
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  if (strcmp(topic, TOPIC_COMMAND) == 0) //komut
    handleCommandMessage(doc);
  else if (strcmp(topic, TOPIC_COORDINATES) == 0) //koordiante
    handleCoordinateMessage(doc);
}

// ============================================================
// COMMAND
// ============================================================
void handleCommandMessage(StaticJsonDocument<256>& doc) {
  String command = doc["command"];

  if (command == "ON") {
    droneDetected = true;
    totalDetections++; //tespit sayisi

    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);

    //lcd.clear();
   // lcd.write(0);
   // lcd.print(" DRONE ALERT");
   // lcd.setCursor(0,1);
    //lcd.print("Count: ");
   // lcd.print(totalDetections);
  }
  else if (command == "OFF") {
    droneDetected = false;

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;

    lcd.clear();
    lcd.print("Status: SAFE");
    lcd.setCursor(0,1);
    lcd.print("Area Clear");
  }
}

// ============================================================
// COORDINATES
// ============================================================
void handleCoordinateMessage(StaticJsonDocument<256>& doc) {
  droneX = doc["x"];
  droneY = doc["y"];
  droneWidth = doc["width"];
  droneHeight = doc["height"];
  droneConfidence = doc["confidence"];
  coordinateUpdates++;
}

// ============================================================
// LCD UPDATE
// ============================================================
void updateLCD() {
  if (!droneDetected) return;
  if (millis() - lastLCDUpdate < LCD_UPDATE_INTERVAL) return;
  lastLCDUpdate = millis();

  //lcdPage = (lcdPage + 1) % 2;
  lcd.clear();


    lcd.write(0);
    lcd.print(" Pos:");
    lcd.print(droneX);
    lcd.print(",");
    lcd.print(droneY);
    lcd.setCursor(0,1);
    lcd.print("Size:");
    lcd.print(droneWidth);
    lcd.print("x");
    lcd.print(droneHeight);
  
    
}

// ============================================================
// BUZZER
// ============================================================
void handleBuzzer() {
  if (!droneDetected) return;

  if (millis() - lastBeepTime >= BEEP_INTERVAL) {
    lastBeepTime = millis();
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
  }
}
