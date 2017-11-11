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
#include <Adafruit_NeoPixel.h>

#define SENSOR_RIGHT_PIN A6
#define SENSOR_LEFT_PIN A1

#define SENSOR_LIGHT_PIN A7

#define PIXEL_PIN 11
#define LED_PIN 13

#define SERVO_RIGHT_PIN 5
#define SERVO_LEFT_PIN 4

#define SPEAKER_PIN 7

#define SONAR_ECHO_PIN A0
#define SONAR_TRIGGER_PIN 12
#define SONAR_SHORT_DELAY 2
#define SONAR_LONG_DELAY 10
#define SONAR_MAX_DISTANCE 100
#define SONAR_MAGIC_NUMBER 58.2

#define SERVER_BAUD 57600

#define COMMAND_GET_IR      2
#define COMMAND_MOVE_SERVOS 3
#define COMMAND_SING        4
#define COMMAND_GET_SONAR   5
#define COMMAND_SET_PIXEL   6
#define COMMAND_GET_LIGHT   7
#define COMMAND_SET_LED     8


class RodiHardware {

  Servo leftMotor;
  Servo rightMotor;

  bool isLeftMotorAttached ;
  bool isRightMotorAttached ;

  Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

public:

  RodiHardware(){
    pinMode(LED_PIN, OUTPUT);
    pinMode(SPEAKER_PIN, OUTPUT);
    pinMode(SONAR_TRIGGER_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);

    isLeftMotorAttached = false;
    isRightMotorAttached = false;

    pixel.begin();
  };

  void setPixel(char red, char green, char blue) {
    pixel.setPixelColor(0, pixel.Color(red, green, blue));
    pixel.show();
  }

  unsigned int getLeftIR() {
    unsigned int sensorLeftState = analogRead(SENSOR_LEFT_PIN);
    return sensorLeftState;
  };

  unsigned int getRightIR() {
    unsigned int sensorRightState = analogRead(SENSOR_RIGHT_PIN);
    return sensorRightState;
  };

  unsigned int getLight() {
    unsigned int sensorLightIntensity = analogRead(SENSOR_LIGHT_PIN);
    return sensorLightIntensity;
  };

  void playTone(unsigned int frequency, unsigned int duration) {
    tone(SPEAKER_PIN, frequency, duration);
  };

  void setLed(char isOn) {
    digitalWrite(LED_PIN, isOn);
  }
  void moveLeftServo(char speed){
    isLeftMotorAttached = moveMotor(leftMotor, isLeftMotorAttached, SERVO_LEFT_PIN, speed, 1);
  };

  void moveRightServo(char speed){
    isRightMotorAttached = moveMotor(rightMotor, isRightMotorAttached, SERVO_RIGHT_PIN, speed, -1);
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

protected:

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

};

RodiHardware rodi;

/**
 * Serial-related functions
 */

// Send a 10-bit integer down the wire, which we need to
// map to two-bytes. Use MSB.
void writeInteger(unsigned int value) {
  Serial.write(highByte(value));
  Serial.write(lowByte(value));
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

void executeSing() {
  unsigned int frequency = readUnsignedIntBlocking();
  unsigned int duration = readUnsignedIntBlocking();
  rodi.playTone(frequency, duration);
};

void executeGetSonar() {
  long sonarDistance = rodi.getSonar();
  Serial.write(sonarDistance);
};

void executeSetPixel() {
  char red = readUnsignedIntBlocking();
  char green = readUnsignedIntBlocking();
  char blue = readUnsignedIntBlocking();
  rodi.setPixel(red, green, blue);
}

void executeMoveServos() {
  char leftSpeed = readUnsignedIntBlocking();
  char rightSpeed = readUnsignedIntBlocking();
  rodi.moveLeftServo(leftSpeed);
  rodi.moveRightServo(rightSpeed);
};

void executeGetIR() {
  unsigned int sensorLeftState = rodi.getLeftIR();
  unsigned int sensorRightState = rodi.getRightIR();
  writeInteger(sensorLeftState);
  writeInteger(sensorRightState);
};

void executeGetLight() {
  writeInteger(rodi.getLight());
};

void executeSetLed() {
  char isOn = readUnsignedIntBlocking();
  rodi.setLed(isOn);
}

int commandByte = 0;

void setup() {
  Serial.begin(SERVER_BAUD);
  digitalWrite(LED_PIN, 1);
}

void loop() {

  if (Serial.available() > 0) {

    commandByte = readUnsignedIntBlocking();

    switch (commandByte) {
      case COMMAND_GET_IR: {
        executeGetIR();
        break;
      }
      case COMMAND_MOVE_SERVOS: {
        executeMoveServos();
        break;
      }
      case COMMAND_SING: {
        executeSing();
        break;
      }
      case COMMAND_GET_SONAR: {
        executeGetSonar();
        break;
      }
      case COMMAND_SET_PIXEL: {
        executeSetPixel();
        break;
      }
      case COMMAND_GET_LIGHT: {
        executeGetLight();
        break;
      }
      case COMMAND_SET_LED: {
        executeSetLed();
        break;
      }
      default: {
        break;
      }
    }
  }
}
