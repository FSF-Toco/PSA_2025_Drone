#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include "Kalman.h"
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- Wi-Fi & MQTT ---
const char* ssid = "hotspot name";
const char* password = "hotspot password";
const char* mqtt_server = "ip hotspot";
WiFiClient espClient;
PubSubClient client(espClient);


// --- ESC Pins ---
const int pinESC_1 = 13;
const int pinESC_2 = 26;
const int pinESC_3 = 14;
const int pinESC_4 = 27;
Servo esc1, esc2, esc3, esc4;

// --- Sensor & Kalman ---
MPU9250_asukiaaa mySensor;
Kalman rollKalman, pitchKalman;
unsigned long lastMicros = 0;

// --- Offsets for Zero ---
float rollOffset = 0;
float pitchOffset = 0;
bool resetRequested = false;

// --- Control Variables ---
float integralMax = 100.0;  

String motorMode = "OFF";
int vmax = 1000;

// --- PID State ---
float Kp = 1.5, Ki = 0.02, Kd = 0.3;
float rollSetpoint = 0.0;
float pitchSetpoint = 0.0;

float rollErrorSum = 0, rollLastError = 0;
float pitchErrorSum = 0, pitchLastError = 0;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  topicStr.trim();

  String valueStr;
  for (unsigned int i = 0; i < length; i++) valueStr += (char)payload[i];

  valueStr.trim();  // Just in case payload includes stray characters

  Serial.print("[MQTT] Topic: ");
  Serial.println(topicStr);
  Serial.print("[MQTT] Payload: ");
  Serial.println(valueStr);

  if (topicStr == "motors/mode") {
    motorMode = valueStr;
    Serial.print(">> Motor Mode Set To: ");
    Serial.println(motorMode);
  }
  else if (topicStr == "motors/vmax") {
    vmax = valueStr.toInt();
    Serial.print(">> vmax updated to: ");
    Serial.println(vmax);
  }
  else if (topicStr == "pid/kp") {
    Kp = valueStr.toFloat();
    Serial.print(">> Kp = "); Serial.println(Kp);
  }
  else if (topicStr == "pid/ki") {
    Ki = valueStr.toFloat();
    Serial.print(">> Ki = "); Serial.println(Ki);
  }
  else if (topicStr == "pid/kd") {
    Kd = valueStr.toFloat();
    Serial.print(">> Kd = "); Serial.println(Kd);
  }
  else if (topicStr == "sensors/reset_orientation") {
    resetRequested = true;
    Serial.println(">> Reset orientation requested.");
  }
  else if (topicStr == "pid/imax") {
    integralMax = valueStr.toFloat();
    Serial.print(">> Integral max cap (imax) = ");
    Serial.println(integralMax);
  }

  
}


void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      client.subscribe("motors/mode");
      client.subscribe("motors/vmax");
      client.subscribe("pid/kp");
      client.subscribe("pid/ki");
      client.subscribe("pid/kd");
      client.subscribe("pid/imax");
      client.subscribe("sensors/reset_orientation");  // ✅ Make sure this line is included
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  esc1.attach(pinESC_1, 1000, 2000);
  esc2.attach(pinESC_2, 1000, 2000);
  esc3.attach(pinESC_3, 1000, 2000);
  esc4.attach(pinESC_4, 1000, 2000);

  // ESC calibration
  delay(3000);
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);
  delay(3500);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3500);
  delay(3000);

  // Sensor init
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  delay(100);
  mySensor.accelUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();
  float roll_init = 180 * atan2(ay, sqrt(ax * ax + az * az)) / PI;
  float pitch_init = 180 * atan2(ax, sqrt(ay * ay + az * az)) / PI;

  rollKalman.setQangle(0.01f); rollKalman.setQbias(0.003f); rollKalman.setRmeasure(0.01f);
  pitchKalman.setQangle(0.01f); pitchKalman.setQbias(0.003f); pitchKalman.setRmeasure(0.01f);
  rollKalman.setAngle(roll_init);
  pitchKalman.setAngle(pitch_init);
  lastMicros = micros();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  float ax = mySensor.accelX(), ay = mySensor.accelY(), az = mySensor.accelZ();
  float gx = mySensor.gyroX(), gy = mySensor.gyroY();

  float roll_raw = 180 * atan2(ay, sqrt(ax * ax + az * az)) / PI;
  float pitch_raw = 180 * atan2(ax, sqrt(ay * ay + az * az)) / PI;

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) * 1e-6;
  lastMicros = nowMicros;

  float roll_filtered = rollKalman.getAngle(roll_raw, gx, dt);
  float pitch_filtered = pitchKalman.getAngle(pitch_raw, gy, dt);

  if (resetRequested) {
    rollOffset = roll_filtered;
    pitchOffset = pitch_filtered;
    resetRequested = false;
    Serial.println("Orientation zeroed");
  }

  String payload = "{\"roll\":" + String(roll_filtered - rollOffset, 2) +
                   ",\"pitch\":" + String(pitch_filtered - pitchOffset, 2) + "}";
  client.publish("sensors/rollpitch", payload.c_str());

  int pwm1 = 1000, pwm2 = 1000, pwm3 = 1000, pwm4 = 1000;
  
  if (motorMode == "OFF") {
    pwm1 = pwm2 = pwm3 = pwm4 = 1000;
  } 
  else if (motorMode == "BASIC") {
    pwm1 = pwm2 = pwm3 = pwm4 = vmax;
    Serial.println(vmax);
  } 
  else if (motorMode == "AUTO") {
    float rollError = rollSetpoint - (roll_filtered - rollOffset);
    rollErrorSum += rollError * dt;
    rollErrorSum = constrain(rollErrorSum, -integralMax, integralMax); // cap na soma integral
    float rollDeriv = (rollError - rollLastError) / dt;
    rollLastError = rollError;
    float rollCorrection = Kp * rollError + Ki * rollErrorSum + Kd * rollDeriv;
    float pitchError = pitchSetpoint - (pitch_filtered - pitchOffset);

    pitchErrorSum += pitchError * dt;
    pitchErrorSum = constrain(pitchErrorSum, -integralMax, integralMax); // cap na soma integral
    float pitchDeriv = (pitchError - pitchLastError) / dt;
    pitchLastError = pitchError;
    float pitchCorrection = Kp * pitchError + Ki * pitchErrorSum + Kd * pitchDeriv;

    int basePWM = vmax;
    pwm1 = basePWM - rollCorrection - pitchCorrection;
    pwm2 = basePWM + rollCorrection - pitchCorrection;
    pwm3 = basePWM + rollCorrection + pitchCorrection;
    pwm4 = basePWM - rollCorrection + pitchCorrection;

    pwm1 = constrain(pwm1, 1000, 2000);
    pwm2 = constrain(pwm2, 1000, 2000);
    pwm3 = constrain(pwm3, 1000, 2000);
    pwm4 = constrain(pwm4, 1000, 2000);
  }

  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  esc3.writeMicroseconds(pwm3);
  esc4.writeMicroseconds(pwm4);
}
