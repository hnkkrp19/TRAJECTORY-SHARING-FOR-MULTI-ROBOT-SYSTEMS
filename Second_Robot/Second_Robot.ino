#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ENCODER PINS
const int ENCODER_LEFT_PIN = 3;
const int ENCODER_RIGHT_PIN = 2;

// NRF24L01
RF24 radio(8, 9); // CE, CSN
const byte diachi[6] = "12345";

struct DataPacket {
    int leftPWM;
    int rightPWM;
    float heading;
    float xPos;
    float yPos;
    float xGoal;
    float yGoal;
}receivedData;

// Kalman Filter
struct KalmanFilter {
    float estimate;
    float errorEstimate;
    float processNoise;
    float measurementNoise;

    KalmanFilter(float processNoise, float measurementNoise) {
        this->processNoise = processNoise;
        this->measurementNoise = measurementNoise;
        this->estimate = 0.0;
        this->errorEstimate = 1.0;
    }

    float update(float measurement) {
        float prioriEstimate = estimate;
        float prioriError = errorEstimate + processNoise;

        // Update
        float kalmanGain = prioriError / (prioriError + measurementNoise);
        estimate = prioriEstimate + kalmanGain * (measurement - prioriEstimate);
        errorEstimate = (1 - kalmanGain) * prioriError;

        return estimate;
    }
};

// Kalman X, Y và heading
KalmanFilter kalmanX(0.01, 0.1);
KalmanFilter kalmanY(0.01, 0.1);
KalmanFilter kalmanHeading(0.01, 0.1);

// timeout
unsigned long lastReceivedTime = 0;
const unsigned long timeoutDuration = 1000;

// Move
bool isMoving = false;

// Variable that stores the number of pulses from the slave encoder
unsigned long leftPulses = 0;
unsigned long rightPulses = 0;
unsigned long prevLeftPulses = 0;
unsigned long prevRightPulses = 0;

// Location slave
float slaveX = 0.0;
float slaveY = 0.0;
float heading = 0.0; 

// Variable to store initial angle and coordinates
bool initialized = false;
float initialHeading = 0.0;

// Function to count pulses from left encoder
void pulseLeft() {
    leftPulses++;// Increase left pulse
}

// Function to count pulses from right encoder
void pulseRight() {
    rightPulses++; // Increase right pulse
}

// update location slave based on encoder data
void updateSlavePosition() {
    // If not initialized, set initial position from master
    if (!initialized) {
        slaveX = receivedData.xPos;
        slaveY = receivedData.yPos;
        heading = receivedData.heading;
        initialized = true;
        Serial.println("Initialize angle and coordinates.");
    }

// Calculate distance traveled from encoder
    float leftDistance = (leftPulses - prevLeftPulses) * DISTANCE_PER_PULSE;
    float rightDistance = (rightPulses - prevRightPulses) * DISTANCE_PER_PULSE;
    float distance = (leftDistance + rightDistance) / 2.0;
    float deltaHeading = (rightDistance - leftDistance) / WHEEL_BASE;

// Update position and rotation angle
    heading += deltaHeading;
    slaveX += distance * cos(heading);
    slaveY += distance * sin(heading);

// Kalman
    slaveX = kalmanX.update(slaveX);
    slaveY = kalmanY.update(slaveY);
    heading = kalmanHeading.update(heading);

// Update previous pulse value
    prevLeftPulses = leftPulses;
    prevRightPulses = rightPulses;

// Log information to check
    Serial.print("Filtered X: ");
    Serial.print(slaveX);
    Serial.print(" | Filtered Y: ");
    Serial.print(slaveY);
    Serial.print(" | Filtered Heading: ");
    Serial.println(heading);
}

// Left motor control function
void controlLeftMotor(int pwmValue) {
    if (pwmValue > 0) {
        analogWrite(LEFT_EN, pwmValue);
        digitalWrite(LEFT1, HIGH);
        digitalWrite(LEFT2, LOW);
    } else {
        analogWrite(LEFT_EN, -pwmValue);
        digitalWrite(LEFT1, LOW);
        digitalWrite(LEFT2, HIGH);
    }
}

// RightRight motor control function
void controlRightMotor(int pwmValue) {
    if (pwmValue > 0) {
        analogWrite(RIGHT_EN, pwmValue);
        digitalWrite(RIGHT1, HIGH);
        digitalWrite(RIGHT2, LOW);
    } else {
        analogWrite(RIGHT_EN, -pwmValue);
        digitalWrite(RIGHT1, LOW);
        digitalWrite(RIGHT2, HIGH);
    }
}

void setup() {
    Serial.begin(9600);

    // NRF24L01
    if (!radio.begin()) {
        Serial.println("Module failed to start...!!");
        while (1) {}
    }

    radio.openReadingPipe(1, diachi);
    radio.setPALevel(RF24_PA_MIN);
    radio.setChannel(80);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();

// Configure motor control pins
    pinMode(LEFT_EN, OUTPUT);
    pinMode(LEFT1, OUTPUT);
    pinMode(LEFT2, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);
    pinMode(RIGHT1, OUTPUT);
    pinMode(RIGHT2, OUTPUT);

// Encoder configuration
    pinMode(ENCODER_LEFT_PIN, INPUT);
    pinMode(ENCODER_RIGHT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, RISING);

    Serial.println("Slave is ready to receive data...");
}

void loop() {
// Check signal received from master
    if (radio.available()) {
        radio.read(&receivedData, sizeof(receivedData));
        lastReceivedTime = millis(); // Update receiving time

// Control the motor according to the received PWM
        controlLeftMotor(receivedData.leftPWM);
        controlRightMotor(receivedData.rightPWM);
        isMoving = true;

// Update slave vehicle position based on encoder
        updateSlavePosition();
    } else {
// If no signal is received within the allowed time, stop the motor
        if (millis() - lastReceivedTime > timeoutDuration) {
// Stop both motors
            analogWrite(LEFT_EN, 0);
            digitalWrite(LEFT1, LOW);
            digitalWrite(LEFT2, LOW);

            analogWrite(RIGHT_EN, 0);
            digitalWrite(RIGHT1, LOW);
            digitalWrite(RIGHT2, LOW);

            if (isMoving) {
                Serial.println("Lost signal from master, stop vehicle!");
                isMoving = false;
            }
        }
    }
}
