#include "arduino_secrets.h"
#include "thingProperties.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>

// OLED display definition
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// HC-SR04 pins
#define TRIG1 5
#define ECHO1 18
#define TRIG2 2
#define ECHO2 4

// Servo pin
#define SERVO_PIN 14
Servo myServo;

// Trash bin depth
#define TRASH_DEPTH_CM 13L

// GPS configuration
#define GPS_RX 32
#define GPS_TX 33
HardwareSerial mySerial(1); 
TinyGPSPlus gps;

bool lidOpen = false;
bool wasNotified = false;
unsigned long lastOpenTime = 0;
const unsigned long lidOpenDuration = 7000; 

#define FULLNESS_SAMPLE_SIZE 10
float fullnessSamples[FULLNESS_SAMPLE_SIZE];
int sampleIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90); 

  // I2C display setup
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED display failed to initialize");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  // GPS setup
  mySerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

void openLid() {
  myServo.write(0);
  lidOpen = true;
  lastOpenTime = millis();
  Serial.println("Lid opened");
}

void closeLid() {
  myServo.write(90);
  lidOpen = false;
  Serial.println("Lid closed");
}

void onLidControlChange() {
  if (lidControl) {
    openLid();
  } else {
    closeLid();
  }
}

void loop() {
  ArduinoCloud.update();

  long handDistance = readDistanceCM(TRIG1, ECHO1);
  if (handDistance > 0 && handDistance <= 4 && !lidOpen) {
    openLid();
    lidControl = true; 
  }

  if (lidOpen && millis() - lastOpenTime > lidOpenDuration) {
    closeLid();
    lidControl = false;
  }

  // Trash level check
  long trashDistance = readDistanceCM(TRIG2, ECHO2);
  if (trashDistance > 0) {
    trashDistance = min(trashDistance, TRASH_DEPTH_CM);
    fullness = 100 - (trashDistance * 100 / TRASH_DEPTH_CM);
    fullness = constrain(fullness, 0, 100);

    fullnessSamples[sampleIndex] = fullness;
    sampleIndex = (sampleIndex + 1) % FULLNESS_SAMPLE_SIZE;

    float sum = 0;
    int count = 0;
    for (int i = 0; i < FULLNESS_SAMPLE_SIZE; i++) {
      if (fullnessSamples[i] > 0) {
        sum += fullnessSamples[i];
        count++;
      }
    }
    if (count > 0) {
      averageFullness = sum / count;
    } else {
      averageFullness = fullness;
    }

    if (fullness >= 70 && !wasNotified) {
      notify = true;
      wasNotified = true;
      Serial.println("⚠️ Trash bin is 70% full! Notification sent.");
    } else {
      notify = false;
    }

    if (fullness < 60 && wasNotified) {
      wasNotified = false;
    }
  }

  // GPS data
  while (mySerial.available() > 0) {
    char c = mySerial.read();
    gps.encode(c);
    Serial.write(c);

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();

      Serial.print("\nLatitude: ");
      Serial.println(lat, 6);
      Serial.print("Longitude: ");
      Serial.println(lng, 6);

      location = Location(lat, lng);
    }
  }

  // OLED display
  display.clearDisplay();
  display.setCursor(0, 20);
  display.print("Fullness:");
  display.setCursor(0, 45);
  display.print(fullness);
  display.print(" %");
  display.display();

  delay(200);
}
