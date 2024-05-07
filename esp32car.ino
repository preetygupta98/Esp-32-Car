#include "BluetoothSerial.h"
#include <Arduino.h>

BluetoothSerial serialBT;

// Ultrasonic sensor pins
const int trigPin = 32;  // Adjust based on your ESP32 pinout
const int echoPin = 33;  // Adjust based on your ESP32 pinout

// Buzzer pin and state
const int buzzerPin = 25; // GPIO for the buzzer
bool buzzerOn = false;    // State of the buzzer

// Bluetooth signal storage
char btSignal;

// Initial Speed
int Speed = 100;

// PWM pins for motor speed control
int enA = 5;
int enB = 23;

// Motor control pins
int IN1 = 22;
int IN2 = 21;
int IN3 = 19;
int IN4 = 18;

// Function declarations
void forward();
void backward();
void left();
void right();
void stop();
void toggleBuzzer();
long readUltrasonicDistance();

void setup() {
    Serial.begin(115200);
    serialBT.begin("ESP32_BT"); // Bluetooth device name

    // Initialize motor control pins
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    stop(); // Ensure all motors are stopped initially

    ledcSetup(0, 5000, 8);
    ledcAttachPin(enA, 0);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(enB, 1);

    // Initialize ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Setup for the buzzer
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially
}

void loop() {
    if (serialBT.available() > 0) {
        btSignal = serialBT.read();
        Serial.print("Received: ");
        Serial.println(btSignal);  // Debug output

        switch (btSignal) {
            case 'B': backward(); break;
            case 'F': forward(); break;
            case 'L': left(); break;
            case 'R': right(); break;
            case 'S': stop(); break;
            case 'V':
            case 'v': toggleBuzzer(); break;
            default:
                if (isdigit(btSignal)) {
                    Speed = map(btSignal - '0', 0, 9, 100, 255);
                    Serial.print("Speed set to: ");
                    Serial.println(Speed);
                }
                break;
        }
    }
}

void forward() {
    ledcWrite(0, Speed);
    ledcWrite(1, Speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Moving forward");
}

void backward() {
    ledcWrite(0, Speed);
    ledcWrite(1, Speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving backward");
}

void left() {
    ledcWrite(0, Speed);
    ledcWrite(1, Speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Turning left");
}

void right() {
    ledcWrite(0, Speed);
    ledcWrite(1, Speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Turning right");
}

void stop() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopping");
}

void toggleBuzzer() {
    buzzerOn = !buzzerOn;
    digitalWrite(buzzerPin, buzzerOn ? HIGH : LOW);
    Serial.print("Buzzer toggled: ");
    Serial.println(buzzerOn ? "ON" : "OFF");
}

long readUltrasonicDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long distance = pulseIn(echoPin, HIGH) / 58.2; // Convert to cm
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
}
