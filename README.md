
# Collision Avoidance Robot with Custom Bluetooth Controller

The objective of this project is to control a robot vehicle using a custom designed controller. Both the Robot and Controller are powered by the MBED LPC1768 and connected by two HC-05 Bluetooth Modules. The Robot using an ultrasonic sensor to detect obstacles within 250 mm of its front. The Bluetooth Communication uses a protocol similar to the "Stop-And-Wait ARQ" Flow Control Protocol when sending distance readings and acknowledgements back and forth between the vehicle and controller during the Collision Avoidance routine.

### DEMO VIDEO:
See a short demo of the robot in action in this [video!](https://www.youtube.com/watch?v=5chwjKlPG6U)

## Parts List 
* Robot
	* Shadow Robot Kit*
	* mBed LPC1768
	* HC-05 Bluetooth Module
	* H-Bridge Motor Drive
	* Reversible DC Motor (x2)
	* HC-SR04 Ultrasonic Sensor
	* Ambient Light Sensor
	* Headlights (Breadboard LEDs)
	* Barrel Jack Adapters (x2)
	* Battery Packs (x2)
		* AA Batteries (x8)
		* Duct Tape
	* Breadboards (x2)

* Controller
	* mBed LPC1768
	* HC-05 Bluetooth Module
	* uLCD-144-G2 128x128 Smart Color LCD
	* Sparkfun Analog Joystick
	* Breadboard Speaker
	* Mini USB Cable
	* Either a battery pack or PC for power
##### * Assembly instuctions for the Shadow Robot Kit can be found [here](https://www.youtube.com/watch?v=aJRYTqZu5OE)

## Wiring
### Controller :
* **HC-05 Bluetooth Module:**
The Bluetooth Module is used to communicate with the robot. Joystick readings are sent until the collision protocol is entered, at which point the robot sends distance readings describing its surroundings and the controller sends acknowledgements back to indicate the readings were received and processed.

	| mBed      | HC-05   |
	| --------- |:--------|
	| p9        | RX      |
	| p10       | TX      |
	| +5V       | VCC     |
	| GND       | GND     |
	| GND       | EN      |

* **uLCD:**
The LCD screen is used to draw the radar view of the robots surroundings upon entering the collision protocol.

	| mBed | uLCD |
	|--|--|
	| p28|RX|
	| p27|TX|
	| +5V|VCC|
	| GND|GND|
	| p11|RES|

* **Analog Joystick:**
The joystick contains two potentiometers which are used to control the robots movements.
	| mBed | Joystick |
	|-----|-----------|
	|p15|Vert|
	|p16|Horz|
	|Vout (3.3v)|VCC|
	|GND|GND|
	|p17|SEL|
### Robot:
* **HC-05 Bluetooth Module:**
The Bluetooth Module is used to communicate with the robot. Joystick readings are sent until the collision protocol is entered, at which point the robot sends distance readings describing its surroundings and the controller sends acknowledgements back to indicate the readings were received and processed.

	| mBed | HC-05 |
	|--|--|
	|p9  |RX   |
	|p10 |TX   |
	|+5V |VCC  |
	|GND |GND  |
	|GND |EN   |

* **Reversible DC Motors:**
Used to move the robot.

	| DC Motor | H-Bridge Driver | mBed |
	|--|--|--|
	|+ (Left)|  A01 |  |
	|- (Left)| A02 |  |
	|+ (Right)|  B01 |  |
	|- (Right)| B02 |  |
	|  | PWMA| p21|
	|  | PWMB|p22|
	|  | AIN1| p15|
	|  | AIN2 | p16|
	|  | BIN1 | p17 |
	|  | BIN2 | p19 |
	|  | GND (ALL 3 ) | GND |
	|  | VCC | Vout (+3.3v) |
	|  | STBY | Vout (+3.3v) |
	|  | VM | +5v (Battery Pack) |

* **HC-SR04 Ultrasonic Sensor:**
The Ultrasonic Sensor is used to control the collision avoidance protocol and to read in radar data to be sent back to the controller over Bluetooth.

	| HC-SR04 | mBed |
	|--|--|
	|Trig| p5 |
	|Echo| p6 |
	|VCC| +5v (Battery Pack)|
	|GND| GND|

* **Automatic Headlights:**

	| Ambient Light Sensor | mBed | Headlights (LEDs) |
	|--|--|--|
	| Vcc | Vout (+3.3v)|  | 
	|GND| GND| -|
	| Sig| p20|  |
	| | p30| + |

# Setting up the HC-05's
Use [this](https://os.mbed.com/users/edodm85/code/HC05_AT_mode/) code to send AT Commands to the HC-05 Bluetooth Modules. See [this]( https://howtomechatronics.com/tutorials/arduino/how-to-configure-pair-two-hc-05-bluetooth-module-master-slave-commands/) arduino tutorial for the commands to configure both the master and slave HC-05. 
##### **NOTE:** The ENABLE pin must be wired to high to enter Command Mode on the HC-05s. After they are configured, the pin should be wired to ground.

## Authors:
This project was designed and prototyped by both myself (Andres Rodriguez) and David Prina as our final project for Embedded Systems Design.

###### This repository is a mirror of the original mBed notebook page I made as part of this project, see the original mBed notebook page [here](https://os.mbed.com/users/Andres013/notebook/collision-avoidance-robot-with-bluetooth-controlle/)
