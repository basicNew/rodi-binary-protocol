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

#define SERVER_BUFFER_BIG 256
#define SERVER_BUFFER_SMALL 32

#define SENSOR_BATTERY_PIN A1

#define SENSOR_RIGHT_PIN A6
#define SENSOR_LEFT_PIN A3

#define LED_PIN 13

#define SERVO_RIGHT_PIN 6
#define SERVO_LEFT_PIN 5
#define SERVO_STOP 0

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
#define COMMAND_GET_BATTERY    5

// 51-100: Use an actuator
#define COMMAND_TURN_LEAD_ON  51
#define COMMAND_TURN_LEAD_OFF 22

Servo leftMotor;
Servo rightMotor;

bool isLeftMotorAttached;
bool isRightMotorAttached;

/**
 * Serial-related functions
 */

// Send a 10-bit integer down the wire, which we need to
// map to two-bytes.
void writeInteger(int value) {
  Serial.write(lowByte(value));
  Serial.write(highByte(value));
};

/**
 * RoDI primitives
 */

void executeGetBothIR() {
  unsigned int sensorLeftState = analogRead(SENSOR_LEFT_PIN);
  unsigned int sensorRightState = analogRead(SENSOR_RIGHT_PIN);
  writeInteger(sensorLeftState);
  writeInteger(sensorRightState);
};

void executeGetLeftIR() {
  unsigned int sensorLeftState = analogRead(SENSOR_LEFT_PIN);
  writeInteger(sensorLeftState);
};

void executeGetRightIR() {
  unsigned int sensorRightState = analogRead(SENSOR_RIGHT_PIN);
  writeInteger(sensorRightState);
};

void executeGetBattery() {
  unsigned int battery = 2 * analogRead(SENSOR_BATTERY_PIN);
  unsigned int mappedBattery = map(battery, 0, 1023, 0, 4200);
  Serial.print(mappedBattery);
};

void executeGetSonar() {
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

  Serial.write(sonarDistance);
};

void executeTurnLeadOn() {
  digitalWrite(LED_PIN, HIGH);
};

void executeTurnLeadOff() {
  digitalWrite(LED_PIN, LOW);
};

/*
void executeMove() {
  // TODO: Read properly CA'2 integers.
  int leftSpeed = Serial.read();
  int rightSpeed = Serial.read();
  isLeftMotorAttached = moveMotor(leftMotor, isLeftMotorAttached, SERVO_LEFT_PIN, leftSpeed);
  isRightMotorAttached = moveMotor(rightMotor, isRightMotorAttached, SERVO_RIGHT_PIN, rightSpeed);
}


void moveMotor(Servo motor, bool isAttached, int pin, int speed) {
  if (speed == 0) {
    motor.detach();
    return false;
  }
  if(!isAttached){
    motor.attach(pin);
  }
  int angularVelocity = map(speed, -100, 100, 0, 180);
  motor.write(constrain(angularVelocity, 0, 180));
}
*/

int commandByte = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(SONAR_TRIGGER_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);

  isLeftMotorAttached = false;
  isRightMotorAttached = false;

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
      case COMMAND_GET_BATTERY: {
        executeGetBattery();
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
    }
  }
}
