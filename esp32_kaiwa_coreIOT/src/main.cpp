#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
// Cấu hình WiFi
constexpr char WIFI_SSID[] = "Wokwi-GUEST";
constexpr char WIFI_PASSWORD[] = "";  // Wokwi WiFi không cần mật khẩu

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

//status devices
bool ledstate = false;
int servoAngle = 0; //rotation angle of servo
//Frequence send data 5 seconds
const int TELEMETRY_SEND_INTERVAL = 5000;
unsigned long lastSendTime = 0;

//object Servo
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
  myServo.attach(Servo_PIN);
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
  //Send data every 5 seconds
  if (millis() - lastSendTime > TELEMETRY_SEND_INTERVAL) {
    float temperature = random(20, 40); //fretend temperature
    float humidity = random(50, 100); //fretend humidity

    char payload[100];
    snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);
    if(mqttClient.publish("v1/devices/me/telemetry", payload)){
      Serial.println("Data sent successfully: " + String(payload));
    }else{
      Serial.println("Error send data!");
    }
    lastSendTime = millis();
  }
  mqttClient.publish("v1/devices/me/attributes", "{\"test\":\"Hello CoreIoT\"}");

  Serial.println("ESP32's still running...");
  delay(1000);
}
