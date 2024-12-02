#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// Wi-Fi settings
const char* ssid = "ASKLANY";
const char* password = "ASKLANY2012";

// Ubidots settings
const char* mqtt_server = "industrial.api.ubidots.com";
const char* token = "BBUS-BYVKbcq9qajsUpnNH09sMPuoHn4gZP";  // Your Ubidots token
const char* device_label = "esp8266_device";  // Your device name in Ubidots
const char* variable_label = "timer";  // Variable that controls the timer

WiFiClient espClient;
PubSubClient client(espClient);

Servo servoMotor;
int buzzerPin = D2;  // Pin for the buzzer
int buttonPin = D3;  // Pin for the stop alarm button

unsigned long lastTime = 0;
unsigned long interval = 60000;  // Time interval between updates (1 minute)

int timerValue = 0;
bool alarmTriggered = false;

// NTPClient settings to get time from the internet
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // NTP server with a 60-second update interval

// Function to reconnect to the MQTT server
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP8266_Client", token, "")) {
      Serial.println("Connected to Ubidots!");
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Connect to MQTT
  client.setServer(mqtt_server, 1883);
  reconnect();

  // Initialize devices
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  servoMotor.attach(D4);  // Servo motor connected to pin D4

  servoMotor.write(0);  // Close the drawer initially

  // Start NTPClient to get time from the internet
  timeClient.begin();

  // Subscribe to the variable
  client.subscribe(("/v1.6/devices/" + String(device_label) + "/" + String(variable_label)).c_str());
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Periodic update
  unsigned long currentMillis = millis();
  if (currentMillis - lastTime >= interval) {
    lastTime = currentMillis;

    // Update NTP time
    timeClient.update();  // Update time from the internet
    Serial.println(timeClient.getFormattedTime());  // Display the current time in the Serial Monitor

    // Check if it's time
    if (timerValue > 0) {
      if (millis() / 1000 % (timerValue * 60) == 0 && !alarmTriggered) { // Convert minutes to seconds
        triggerAlarm();  // Trigger the alarm
      }
    }
  }

  // Check stop button
  if (digitalRead(buttonPin) == LOW && alarmTriggered) {
    stopAlarm();  // Stop the alarm
  }
}

// Trigger the alarm
void triggerAlarm() {
  Serial.println("Alarm Triggered!");
  digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
  servoMotor.write(90);           // Open the drawer
  alarmTriggered = true;
}

// Stop the alarm
void stopAlarm() {
  Serial.println("Alarm Stopped!");
  digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
  servoMotor.write(0);            // Close the drawer
  alarmTriggered = false;
}

// Handle data received from Ubidots
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Compare topic with variables
  if (strcmp(topic, ("/v1.6/devices/" + String(device_label) + "/" + String(variable_label)).c_str()) == 0) {
    timerValue = message.toInt();  // Assign timer value from Ubidots
    Serial.print("Timer value: ");
    Serial.println(timerValue);  // Display the received value (in minutes)
  }
}