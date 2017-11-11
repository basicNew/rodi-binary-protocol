rodi-binary-protocol
====================

A binary protocol to interact with Gary Servin's RoDi (Robot Didáctico inalámbrico).

Clients for [Pharo Smalltalk](http://pharo.org/), [Ruby](https://www.ruby-lang.org) and [NodeJs](https://nodejs.org) are on their way.

Please consider that this protocol is in a very early stage so big changes should be expected.

### Install

- Clone this repository.
- Clone the [rodi-code repository](https://github.com/rodibot/rodi-code), you will need this to transfer the server to the robot.
- Get the Arduino compiler (https://www.arduino.cc/en/Main/Software).
- Connect to the RoDI access point. Be sure to set in your machine a static IP address in the proper network (e.g. `192.168.4.2`).
- Open the Arduino compiler and load the `server/server.ino` file.
- Before compiling make sure that you have selected Tools -> Board -> Arduino Pro or Pro Mini and Tools -> Processor -> ATMega168(3.3V, 8MHz).
- Click on "Verify" to compile the server. By this time you should have the compiled program stored in `/tmp/build<XYZ>.tmp/server.cpp.hex`
- Use the rodi-code tools to transfer the program to the robot by doing `rodi-code/programmer/program_rodi.py /tmp/build<XYZ>.tmp/server.cpp.hex`

## API description

This library implements a binary protocol, where every command is specified by one byte and followed by one or more bytes according to the required parameters (if any).

### Sensors

#### Read the two IR sensors
- **Description**: Reads the two IR sensors placed at the bottom of the robot.
- **Command code**: `1`
- **Parameters**: None.
- **Returns**: Two numbers from 0 to 1024, sent as four bytes (2-bytes each number) in MSB order.

#### Read left IR sensor
- **Description**: Reads the left IR sensor placed at the bottom of the robot.
- **Command code**: `2`
- **Parameters**: None.
- **Returns**: Number from 0 to 1024, sent as two bytes in MSB order.

#### Read right IR sensor
- **Description**: Reads the right IR sensor placed at the bottom of the robot.
- **Command code**: `3`
- **Parameters**: None.
- **Returns**: Number from 0 to 1024, sent as two bytes in MSB order.

#### Read light sensor
- **Description**: Reads the light sensor placed at the bottom of the robot.
- **Command code**: `5`
- **Parameters**: None.
- **Returns**: Number from 0 to 1024, sent as two bytes in MSB order.

#### Read sonar
- **Description**: Reads the sonar placed at the front of the robot.
- **Command code**: `4`
- **Parameters**: None.
- **Returns**: Number from 0 to 100, sent as one byte.

### Actions

#### Turn led on
- **Description**: Turns the led on
- **Command code**: `51`
- **Parameters**: None.
- **Returns**: Nothing.

#### Turn led off
- **Description**: Turns the led off
- **Command code**: `52`
- **Parameters**: None.
- **Returns**: Nothing.

#### Move left servo
- **Description**: Sets the left servo speed to the provided value.
- **Command code**: `53`
- **Parameters**: The servo speed in the range of `[-100, 100]`. This number must be sent in one byte and will be interpreted in two's complement.
- **Returns**: Nothing.

#### Move right servo
- **Description**: Sets the right servo speed to the provided value.
- **Command code**: `54`
- **Parameters**: The servo speed in the range of `[-100, 100]`. This number must be sent in one byte and will be interpreted in two's complement.
- **Returns**: Nothing.

#### Move both servos
- **Description**: Sets the speed of both the left and right servos.
- **Command code**: `55`
- **Parameters**: Two numbers defining the servo speeds in the range of `[-100, 100]`. These numbers must be sent in one byte each and will be interpreted in two's complement.
- **Returns**: Nothing.

#### Play tone
- **Description**: Plays the given frequency in a buzzer. For more details see https://www.arduino.cc/en/Reference/Tone.
- **Command code**: `56`
- **Parameters**: The frequency of the squared wave that will be generated. This must be sent in a two-byte, which will be interpreted as unsigned, MSB.
- **Returns**: Nothing.

#### Clear tone
- **Description**: Stops playing the sound in the buzzer (if any).
- **Command code**: `57`
- **Parameters**: None.
- **Returns**: Nothing.

