#include "L293.h"
#include "ArduinoBlue.h"
#include "PreMo.h"
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// BLUETOOTH PINS
const int BLUETOOTH_TX = 11;
const int BLUETOOTH_RX = 12;

// ENCODER PINS
const int ENCODER_LEFT_PIN = 3;
const int ENCODER_RIGHT_PIN = 2;

// MAXIMUM MOTOR SPEED VALUE
const int MOTOR_SPEED_MAX = 255;

// SERIAL COMMUNICATION BAUD RATE
const unsigned long BAUD_RATE = 115200;

// ENCODER PULSES
unsigned long leftPulses;
unsigned long rightPulses;

// TIMING VARIABLES
unsigned long prevLeftTime;
unsigned long prevRightTime;
unsigned long prevSendTime;

// PATH FOLLOWING SPEED
const int PATH_FOLLOW_SPEED = 100;

// Variable to store current PWM value
int currentLeftPWM = 0;
int currentRightPWM = 0;

// Variable to track movement status
bool isMoving = false;

// MOTOR CONTROL
L293 leftMotor(LEFT_EN, LEFT1, LEFT2);
L293 rightMotor(RIGHT_EN, RIGHT1, RIGHT2);

// Modes and controls
int MANUAL_MODE = 1;
int AUTO_MODE = 2;
int STOP_ROBOT = 0;
int LIFT = 3;

int mode = MANUAL_MODE; // Set default mode
int button = 0; // Declare the button variable

// Declare motor control functions in advance
void setLeftForward(int speed);
void setLeftReverse(int speed);
void setRightForward(int speed);
void setRightReverse(int speed);
void stopMotors();

// BLUETOOTH ARDUINO BLUE
SoftwareSerial bluetoothSerial(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue bluetooth(bluetoothSerial);

// PATH FOLLOWER
MotorManager motorManager(setLeftForward, setLeftReverse, setRightForward, setRightReverse, stopMotors);
EncoderManager encoderManager(&leftPulses, &rightPulses, PULSES_PER_REV);
PreMo premo(RADIUS, LENGTH, KP, KD, KP_motor, KI_motor, &motorManager, &encoderManager);

// NRF24L01
RF24 radio(9, 10); // CE, CSN
const byte diachi[6] = "12345";

// Data packet structure sent to slave
struct DataPacket {
    int leftPWM;
    int rightPWM;
    float heading;
    float xPos;
    float yPos;
    float xGoal;
    float yGoal;
};
// MOTOR SPEED FUNCTIONS
void setLeftForward(int speed) {
    currentLeftPWM = speed;
    leftMotor.forward(speed);
    isMoving = true;
}

void setLeftReverse(int speed) {
    currentLeftPWM = speed;
    leftMotor.back(speed);
    isMoving = true;
}

void setRightForward(int speed) {
    currentRightPWM = speed;
    rightMotor.forward(speed);
    isMoving = true;
}

void setRightReverse(int speed) {
    currentRightPWM = speed;
    rightMotor.back(speed);
    isMoving = true;
}

void stopMotors() {
    currentLeftPWM = 0;
    currentRightPWM = 0;
    leftMotor.stop();
    rightMotor.stop();
    isMoving = false;
}

// Encoder interrupt control
void pulseLeft() {
    if (digitalRead(ENCODER_LEFT_PIN) && micros() - prevRightTime > MIN_PULSE_INTERVAL) {
        leftPulses++;
        prevRightTime = micros();
    }
}

void pulseRight() {
    if (digitalRead(ENCODER_RIGHT_PIN) && micros() - prevLeftTime > MIN_PULSE_INTERVAL) {
        rightPulses++;
        prevLeftTime = micros();
    }
}

void attachInterrupts() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

void detachInterrupts() {
    detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
    detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
}

void setupStuff() {
    Serial.begin(BAUD_RATE);
    bluetoothSerial.begin(BAUD_RATE);
    bluetooth.setInterruptToggle(attachInterrupts, detachInterrupts);
    motorManager.setSpeedLimits(0, MOTOR_SPEED_MAX);
    premo.twistBothMotors(false);
    premo.setPathFollowSpeed(PATH_FOLLOW_SPEED);

    attachInterrupts();

// Install NRF24L01
    if (!radio.begin()) {
        Serial.println("Module failed to start...!!");
        while (1) {}
    }

    radio.openWritingPipe(diachi);
    radio.setPALevel(RF24_PA_MIN);
    radio.setChannel(80);
    radio.setDataRate(RF24_250KBPS);
    radio.stopListening();
}

// Send PWM and trajectory data via NRF24L01 and Bluetooth
void sendLocation() {
    if (isMoving && millis() - prevSendTime > SEND_INTERVAL) {
        float xPos = premo.getX();
        float yPos = premo.getY();
        float heading = premo.getHeading();
        float xGoal = premo.getGoalX();
        float yGoal = premo.getGoalY();

        DataPacket packet;
        packet.leftPWM = currentLeftPWM;
        packet.rightPWM = currentRightPWM;
        packet.heading = heading;
        packet.xPos = xPos;
        packet.yPos = yPos;
        packet.xGoal = xGoal;
        packet.yGoal = yGoal;

        // Send data packet via NRF24L01
        radio.write(&packet, sizeof(packet));

        // Send location via Bluetooth to see where the car is
        bluetooth.sendLocation(xPos, yPos, heading, xGoal, yGoal);
    }
}

void pathFollowing() {
    if (bluetooth.isPathAvailable()) {
        delay(1000);
        float* pathX = bluetooth.getPathArrayX();
        float* pathY = bluetooth.getPathArrayY();
        int pathLength = bluetooth.getPathLength();
        premo.startPathFollowing(pathX, pathY, pathLength);
    }

    if (premo.isFollowingPath()) {
        sendLocation();
    }
}

void loopStuff() {
    premo.loop();
    bluetooth.checkBluetooth();
    button = bluetooth.getButton();
}

void checkModes() {
    if (button == MANUAL_MODE) {
        mode = MANUAL_MODE;
        Serial.println("MANUAL");
    } else if (button == AUTO_MODE) {
        mode = AUTO_MODE;
        Serial.println("AUTO");
    }
}

void setup() {
    setupStuff();
    Serial.println("SETUP COMPLETE: Path Follower");
}

void loop() {
    loopStuff();
    checkModes();

    if (button == STOP_ROBOT) {
        premo.stop();
        stopMotors();
        Serial.println("The robot has stopped");
    }

    if (mode == MANUAL_MODE) {
        pathFollowing();
        handleSteering();
    } else if (mode == AUTO_MODE) {
        sendLocation();
    }
    sendLocation();
}
