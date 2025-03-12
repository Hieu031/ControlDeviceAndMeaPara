#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <DHT.h>

#define RELAY_PIN 26
#define BUZZER_PIN 27

//init functions
void checkTemperature();
void sendTelemetryData();
void sendPumpStatus();

// Cấu hình WiFi
constexpr char WIFI_SSID[] = "Wokwi-GUEST";
constexpr char WIFI_PASSWORD[] = "";  // As simulate by Wokwi, so not need password.

// Cấu hình CoreIoT MQTT
constexpr char TOKEN[] = "jchkq8hjfq0xbmwwhdmr";
constexpr char COREIOT_SERVER[] = "app.coreiot.io";
constexpr uint16_t COREIOT_PORT = 1883U;

//mqtt client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//Define Led pin
constexpr int LED_PIN = 2;
constexpr int Servo_PIN = 4;
constexpr int DHT_PIN = 5;
constexpr int DHT_TYPE = DHT22;

//Temperature warning
const float TEMP_THRESHOLD = 35.0; //turn on warning
const float TEMP_HYSTERESIS = 33.0; //turn off pump

//status devices
bool pumpControl = false;
bool pumpstate = false;
bool ledstate = false;
int servoAngle = 0; //rotation angle of servo

//Frequence send data 2 seconds
const int TELEMETRY_SEND_INTERVAL = 2000;
unsigned long lastSendTime = 0;
//buzzer sound 1s
unsigned long buzzerStartTime = 0;
const int BUZZER_SOUND = 1000;
bool isBuzzerOn = false;

//object Servo & DHT22
DHT dht(DHT_PIN, DHT_TYPE);
Servo myServo;

//Connect wifi
void InitWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

//Control led via CoreIOT
void callback(char* topic, byte* payload, unsigned int length){
  Serial.print("Received message on topic: ");
  Serial.println(topic);

  String message;
  for (unsigned int i = 0; i < length; i++){
    message += (char)payload[i];
  }

  Serial.print("Messege: ");
  Serial.println(message);

  int request_id = atoi(strrchr(topic, '/') + 1); //get request ID
  //control pump
  if(message.indexOf("\"method\":\"setpump\"") != -1){
    if (message.indexOf("\"params\":true") != -1){
      pumpstate = true;//turn on Pump
      pumpControl = true;//point mode control 
      Serial.println("PUMP ON");
    }
    else if(message.indexOf("\"params\":false") != -1){
      pumpstate = false;//turn off pump
      pumpControl = false; //return mode auto
      Serial.println("PUMP OFF");
    }

    sendPumpStatus();
  }

  //control Led
  if(message.indexOf("\"method\":\"setled\"") != -1){
    if (message.indexOf("\"params\":true") != -1){
      ledstate = true;
      Serial.println("LED ON");
    }
    else if(message.indexOf("\"params\":false") != -1){
      ledstate = false;
      Serial.println("LED OFF");
    }
  }

  //control servo
  if(message.indexOf("\"method\":\"setservo\"") != -1){
    int startIndex = message.indexOf("\"params\":") + 9;
    int endIndex = message.indexOf("}", startIndex);
    if (startIndex != -1 && endIndex != -1){
      String angleStr = message.substring(startIndex, endIndex);
      servoAngle = angleStr.toInt();
      servoAngle = constrain(servoAngle, 0, 180); //limit rotation 0 to 180 
    Serial.print("Servo Angle: ");
    Serial.println(servoAngle);
    }
  }
  //updateDevices();

  //send respone to CoreIOT
  char responseTopic[50];
  snprintf(responseTopic, sizeof(responseTopic), "v1/devices/me/rpc/response/%d", request_id);
  mqttClient.publish(responseTopic, "{\"status\":\"success\"}");
}
 

//update status of devices
void updateDevices(){
  digitalWrite(RELAY_PIN, pumpstate ? HIGH : LOW);
  digitalWrite(LED_PIN, ledstate ? HIGH : LOW);
  myServo.write(servoAngle);
}

//Reconnect MQTT 
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT Broker... ");
    mqttClient.setServer(COREIOT_SERVER, COREIOT_PORT);

    if (mqttClient.connect("ESP32_CLIENT", TOKEN, "")) {
      Serial.println("Connected to MQTT Broker!");
      mqttClient.subscribe("v1/devices/me/rpc/request/+");
      Serial.println("Subcribed to RPC topic.");
      //send attributes Wifi up CoreIOT
      char attrPayload[200];
      snprintf(attrPayload, sizeof(attrPayload),
              "{\"mac_address\": \"%s\", \"ip_address\": \"%s\", \"ssid\": \"%s\", \"bssid\": \"%s\", \"channel\": %d}",
              WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
              WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(), WiFi.channel());

              if (mqttClient.publish("v1/devices/me/attributes", attrPayload)){
                Serial.println("Sent WiFi attributes successfully!");
              }
              else{
                Serial.println("Failed to sent WiFi attributes!");
              }
    } else {
      Serial.print("Failed, error code: ");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  myServo.attach(Servo_PIN);
  dht.begin();
  delay(1000);
  Serial.println("ESP32 is running in Wokwi!\n");
  InitWiFi();
  mqttClient.setCallback(callback);
  reconnectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) InitWiFi();
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  mqttClient.loop();

  updateDevices();
  //checkTemperature();
  //Send data every 5 seconds
  if (millis() - lastSendTime > TELEMETRY_SEND_INTERVAL) {
      checkTemperature();
      sendTelemetryData();
      lastSendTime = millis();
  }
  mqttClient.publish("v1/devices/me/attributes", "{\"test\":\"Hello CoreIoT\"}");

  Serial.println("ESP32's still running...");
  delay(1000);
}

void checkTemperature(){
  float temperature = dht.readTemperature();
  if (isnan(temperature)){
    Serial.println("Erorr reading from DHT22 sensor!");
    return;
  }

  if (!pumpControl){
    if (temperature > TEMP_THRESHOLD){
      digitalWrite(RELAY_PIN, LOW); //turn on Pump
      pumpstate = true;
      if(isBuzzerOn){
        digitalWrite(BUZZER_PIN, HIGH); //turn on buzzer
        buzzerStartTime = millis(); //save time turn buzzer
        isBuzzerOn = true;
      }  
      Serial.println("Waning: Temperature exceeds permission! Pump turning on.");
    }
    else if (temperature < TEMP_HYSTERESIS) {
      digitalWrite(RELAY_PIN, HIGH);
      pumpstate = false;
      Serial.println("Temperature back normal! Turn off pump.");
    }

    sendPumpStatus();
  }

  //Check buzzer sound 1s!
  if (isBuzzerOn && millis() - buzzerStartTime >= BUZZER_SOUND){
    digitalWrite(BUZZER_PIN, LOW);
    isBuzzerOn = false;
  }
}

void sendTelemetryData(){
  float temperature = dht.readTemperature(); //read temperature 
    //random(20, 40); //fretend temperature
  float humidity = dht.readHumidity(); //read humidity
  //random(50, 100); //fretend humidity
  if (isnan(temperature) || isnan(humidity)){
    Serial.println("Erorr reading from DHT22 sensor!");}
  else{
    char payload[100];
    snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);
    if(mqttClient.publish("v1/devices/me/telemetry", payload)){
      Serial.println("Data sent successfully: " + String(payload));
    }else{
      Serial.println("Error send data!");
    }
  }
}

void sendPumpStatus() {
  char payload[50];
  snprintf(payload, sizeof(payload), "{\"pumpstate\": %s}", pumpstate ? "true" : "false");
  mqttClient.publish("v1/devices/me/attributes", payload);
  Serial.println("Sent pump status to CoreIoT: " + String(payload));
}
