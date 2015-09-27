/*
  Copyright (C) 2015 Andrés Fortier - andres@creativa77.com
  Based on the work of Abente Lahaye (tch@sugarlabs.org) in https://github.com/rodibot/rodi-code.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
  USA
 */

#include <Servo.h>

#define SENSOR_RIGHT_PIN A6
#define SENSOR_LEFT_PIN A3

#define LED_PIN 13

#define SERVO_RIGHT_PIN 6
#define SERVO_LEFT_PIN 5

#define SPEAKER_PIN 2

#define SONAR_ECHO_PIN A0
#define SONAR_TRIGGER_PIN A2
#define SONAR_SHORT_DELAY 2
#define SONAR_LONG_DELAY 10
#define SONAR_MAX_DISTANCE 100
#define SONAR_MAGIC_NUMBER 58.2

#define SERVER_BAUD 57600

// 1-50: Get sensor information
#define COMMAND_GET_BOTH_IR    1
#define COMMAND_GET_LEFT_IR    2
#define COMMAND_GET_RIGHT_IR   3
#define COMMAND_GET_SONAR      4
// In the TODO list:
// #define COMMAND_GET_BATTERY    5

// 51-100: Perform low-level actions
#define COMMAND_TURN_LEAD_ON     51
#define COMMAND_TURN_LEAD_OFF    52
#define COMMAND_MOVE_LEFT_SERVO  53
#define COMMAND_MOVE_RIGHT_SERVO 54
#define COMMAND_MOVE_SERVOS      55
#define COMMAND_PLAY_TONE        56
#define COMMAND_CLEAR_TONE       57


// 101-150: Publish-subscribe
// TBD

// 151-200: High-level commands
// TBD


struct RodiHardware {


  Servo leftMotor;
  Servo rightMotor;

  bool isLeftMotorAttached ;
  bool isRightMotorAttached ;

  void initialize(){
    pinMode(LED_PIN, OUTPUT);
    pinMode(SPEAKER_PIN, OUTPUT);
    pinMode(SONAR_TRIGGER_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);

    isLeftMotorAttached = false;
    isRightMotorAttached = false;
  };

  void turnLeadOn() {
  digitalWrite(LED_PIN, HIGH);
  };

  void turnLeadOff() {
    digitalWrite(LED_PIN, LOW);
  };
  
  unsigned int getLeftIR() {
    unsigned int sensorLeftState = analogRead(SENSOR_LEFT_PIN);
    return sensorLeftState;
  };
  
  unsigned int getRightIR() {
    unsigned int sensorRightState = analogRead(SENSOR_RIGHT_PIN);
    return sensorRightState;
  };

  void playTone(unsigned int frequency) {
    tone(SPEAKER_PIN, frequency);
  };
  
  void clearTone() {
    noTone(SPEAKER_PIN);
  };

  void moveLeftServo(char speed){
    isLeftMotorAttached = moveMotor(leftMotor, isLeftMotorAttached, SERVO_LEFT_PIN, speed, 1);
  }  

  void moveRightServo(char speed){
    isRightMotorAttached = moveMotor(rightMotor, isRightMotorAttached, SERVO_RIGHT_PIN, speed, -1);
  } 
   
  bool moveMotor(Servo motor, bool isAttached, int pin, int speed, int sign) {
    if (speed == 0) {
      motor.detach();
      return false;
    }
    if(!isAttached){
      motor.attach(pin);
    }
    int angularVelocity = map(speed, -100 * sign, 100 * sign, 0, 180);
    motor.write(constrain(angularVelocity, 0, 180));
    return true;
  };

  long getSonar() {
    digitalWrite(SONAR_TRIGGER_PIN, LOW);
    delayMicroseconds(SONAR_SHORT_DELAY);
  
    digitalWrite(SONAR_TRIGGER_PIN, HIGH);
    delayMicroseconds(SONAR_LONG_DELAY);
  
    digitalWrite(SONAR_TRIGGER_PIN, LOW);
    float sonarDuration = (float) pulseIn(SONAR_ECHO_PIN, HIGH);
    long sonarDistance = sonarDuration / SONAR_MAGIC_NUMBER;
  
    if (sonarDistance > SONAR_MAX_DISTANCE) {
      sonarDistance = SONAR_MAX_DISTANCE;
    }
  
    return sonarDistance;
  };

};

RodiHardware rodiHardware;

/**
 * Serial-related functions
 */

// Send a 10-bit integer down the wire, which we need to
// map to two-bytes. Use MSB.
void writeInteger(unsigned int value) {
  Serial.write(highByte(value));
  Serial.write(lowByte(value));
};

// Read a byte from the serial port, blocking until that
// byte is read.
int readByteBlocking() {
  while (!Serial.available());
  return Serial.read();
};

// Read two bytes from the serial port, blocking until
// they are available. Interpret the bytes as an unsigned
// int, reading in MSB order.
unsigned int readUnsignedIntBlocking() {
  while (Serial.available() < 2);
  unsigned int value = Serial.read() << 8;
  value += Serial.read();
  return value;
};

/**
 * RoDI primitives
 */
void executeTurnLeadOn() {
  rodiHardware.turnLeadOn();
};

void executeTurnLeadOff() {
  rodiHardware.turnLeadOff();
};

void executeGetSonar() {
  long sonarDistance = rodiHardware.getSonar();
  Serial.write(sonarDistance);
};

void executeMoveLeftServo() {
  char speed = readByteBlocking();
  rodiHardware.moveLeftServo(speed);
};

void executeMoveRightServo() {
  char speed = readByteBlocking();
  rodiHardware.moveRightServo(speed);
};

void executeMoveServos() {
  char leftSpeed = readByteBlocking();
  char rightSpeed = readByteBlocking();
  rodiHardware.moveLeftServo(leftSpeed);
  rodiHardware.moveRightServo(rightSpeed);
};

void executeGetBothIR() {
  unsigned int sensorLeftState = rodiHardware.getLeftIR();
  unsigned int sensorRightState = rodiHardware.getRightIR();
  writeInteger(sensorLeftState);
  writeInteger(sensorRightState);
};

void executeGetLeftIR() {
  writeInteger(rodiHardware.getLeftIR());
};

void executeGetRightIR() {
  writeInteger(rodiHardware.getRightIR());
};

void executePlayTone() {
  unsigned int frequency = readUnsignedIntBlocking();
  rodiHardware.playTone(frequency);
};

void executeClearTone() {
  rodiHardware.clearTone();
};

int commandByte = 0;

void setup() {
  
  rodiHardware.initialize();
  Serial.begin(SERVER_BAUD);
}

void loop() {
  if (Serial.available() > 0) {
    commandByte = Serial.read();
    switch (commandByte) {

      case COMMAND_GET_BOTH_IR: {
        executeGetBothIR();
        break;
      }
      case COMMAND_GET_LEFT_IR: {
        executeGetLeftIR();
        break;
      }
      case COMMAND_GET_RIGHT_IR: {
        executeGetRightIR();
        break;
      }
      case COMMAND_GET_SONAR: {
        executeGetSonar();
        break;
      }
      case COMMAND_TURN_LEAD_ON: {
        executeTurnLeadOn();
        break;
      }
      case COMMAND_TURN_LEAD_OFF: {
        executeTurnLeadOff();
        break;
      }
      case COMMAND_MOVE_LEFT_SERVO: {
        executeMoveLeftServo();
        break;
      }
      case COMMAND_MOVE_RIGHT_SERVO: {
        executeMoveRightServo();
        break;
      }
      case COMMAND_MOVE_SERVOS: {
        executeMoveServos();
        break;
      }
      case COMMAND_PLAY_TONE: {
        executePlayTone();
        break;
      }
      case COMMAND_CLEAR_TONE: {
        executeClearTone();
        break;
      }
    }
  }
}
