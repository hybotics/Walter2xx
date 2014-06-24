/*
	W.A.L.T.E.R. 2.0: Motor and Controller Test sketch.
	Copyright (C) 2014 Dale A. Weber <hybotics.pdx@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
	Program:		W.A.L.T.E.R. 2.0, Motor and Controller Test sketch
	Date:			24-Jun-2014
	Version:		0.1.0 Arduino Mega ADK - ALPHA

	Purpose:		Test Motors and Motor Controllers
						
	Dependencies:	Adafruit libraries:
						Adafruit_Sensor and Adafruit_L3GD20 libraries.

					Hybotics libraries:
						Hybotics_10DOF_Unified (forked from the Adafruit_10DOF library)
						Hybotics_LSM303DLHC_Unified (forked from the Adafruit_LSM303 library)

					Other libraries:
						RTClib for the DS1307 (Adafruit version),
						KalmanFilter

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
	    				Playground, which I have modified to suit my needs.
*/

#include <Wire.h>

#include <RTClib.h>

#include <BMSerial.h>
#include <RoboClaw.h>


/*
	Additional libraries
*/

#include "Motor_Test.h"
#include "Pitches.h"

/********************************************************************/
/*	Global objects													*/
/********************************************************************/

RTC_DS1307 clock;

/********************************************************************/
/*	Initialize global variables										*/
/********************************************************************/

/*
	Time control variables
*/
uint8_t currentMinute = 0;
uint8_t lastMinute = -1;
long minuteCount = 0;						//	Count the time, in minutes, since we were last restarted

//	Enable run once only loop initialization code to run for special cases
bool firstLoop = true;

/*
	Setup all our serial devices
*/

//	Hardware Serial: Console and debug (replaces Serial.* routines)
BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN, 10000);

//	Hardware Serial1: RoboClaw 3x5 Motor Controller
RoboClaw roboClaw(SERIAL_ROBOCLAW_RX_PIN, SERIAL_ROBOCLAW_TX_PIN, 10000, false);

//	Hardware Serial2: SSC-32 Servo Controller
BMSerial ssc32(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);

//	Hardware Serial3: XBee Mesh Wireless
BMSerial xbee(SERIAL_XBEE_RX_PIN, SERIAL_XBEE_TX_PIN);

//	We only have one RoboClaw 2x5 right now
uint8_t roboClawControllers = ROBOCLAW_CONTROLLERS - 1;
uint8_t	roboClawBaseAddress = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress1 = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress2 = ROBOCLAW_SERIAL_BASE_ADDR + 1;

//	Left side motor (M1) - RoboClaw 2x5 Controller #1 (0x80)
Motor leftMotorM1;

//	Right side motor (M2) - RoboClaw 2x5 Controller #1 (0x80)
Motor rightMotorM2;

//	Front motor (M1) - RoboClaw 2x5 Controller #2 (0x81)
Motor frontMotorM1;

//	Back motor (M2) - RoboClaw 2x5 Controller #2 (0x81)
Motor backMotorM2;

/*
	Code starts here
*/

/********************************************************************/
/*	Utility routines 												*/
/********************************************************************/

/*
    Left zero pad a numeric string
*/
String leftZeroPadString (String st, uint8_t nrPlaces) {
	uint8_t i, len;
	String newStr = st;
  
	if (newStr.length() < nrPlaces) {
		len = st.length();
  
		for (i = len; i < nrPlaces; i++) {
			newStr = String("0" + newStr);
		}
	}

	return newStr;
}

/*
    Convert a pulse width in ms to inches
*/
long microsecondsToInches (long microseconds) {
	/*
		According to Parallax's datasheet for the PING))), there are
			73.746 microseconds per inch (i.e. sound travels at 1130 feet per
			second).  This gives the distance travelled by the ping, outbound
			and return, so we divide by 2 to get the distance of the obstacle.

		See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
	*/
	
	return microseconds / 74 / 2;
}

/*
    Convert a pulse width in ms to a distance in cm
*/
long microsecondsToCentimeters (long microseconds) {
	/*
		The speed of sound is 340 m/s or 29 microseconds per centimeter.

		The ping travels out and back, so to find the distance of the
			object we take half of the distance travelled.
	*/

	return microseconds / 29 / 2;
}

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
	digitalWrite(pin, HIGH);			// Turn the ON by making the voltage HIGH (5V)
	delay(duration);					// Wait for duration ms
	digitalWrite(pin, LOW);				// Turn the pin OFF by making the voltage LOW (0V)
	delay(duration);					// Wait for duration ms
}

/*
	Convert a temperature in Celsius to Fahrenheit
*/
float toFahrenheit (float celsius) {
	return (celsius * 1.8) + 32;
}

/*
    Trim trailing zeros from a numeric string
*/
String trimTrailingZeros (String st) {
	uint8_t newStrLen = 0;
	String newStr = st;

	newStrLen = newStr.length();

	while (newStr.substring(newStrLen - 1) == "0") {
		newStrLen -= 1;
		newStr = newStr.substring(0, newStrLen);
	}

	return newStr;
}

/********************************************************************/
/*	Display routines 												*/
/********************************************************************/

/*
	Display the data for a given motor
*/
void displayMotor (Motor *motor, String name) {
	console.print(motor->name);
	console.println(F(" Motor:"));

	//	Using Packet Serial
	console.print(F("Encoder is valid: "));

	if (motor->encoderValid) {
		console.print(F("Yes, Status: "));
		console.print(motor->encoderStatus);
		console.print(F(", Value: "));
		console.println(motor->encoder);
	} else {
		console.println(F("No"));
	}

	console.print(F("Speed is valid: "));

	if (motor->speedValid) {
		console.print(F("Yes, Status: "));
		console.print(motor->speedStatus);
		console.print(F(", Speed: "));
		console.println(motor->speed);
	} else {
		console.println(F("No"));
	}

	console.print(F("Moving "));

	if (motor->forward) {
		console.print(F("Forward"));
	} else {
		console.print(F("Reverse"));
	}

	console.print(F("Distance is valid: "));

	if (motor->distanceValid) {
		console.print(F("Yes, Distance: "));
		console.println(motor->distance);
	} else {
		console.println(F("No"));
	}
}

/*
	Display data from the RoboClaw 2x5 motor controller
*/
void displayRoboClawData (uint8_t address, Motor *leftMotorM1, Motor *rightMotorM2) {
	char version[32];

	roboClaw.ReadVersion(address, version);

	console.print(F("RoboClaw 2x5 status (version "));
	console.print(version);
	console.print(F("): "));
	console.println();

    if (leftMotorM1->encoderValid) {
		console.print(F("Left Motor Encoder = "));
		console.print(leftMotorM1->encoder, DEC);
		console.print(F(", Status =  "));
		console.print(leftMotorM1->encoderStatus, HEX);
		console.println();
	}

	if (leftMotorM1->speedValid) {
		console.print(F("Left Motor Speed = "));
		console.print(leftMotorM1->speed, DEC);
		console.println();
	}

	if (rightMotorM2->encoderValid) {
		console.print(F("Right Motor Encoder = "));
		console.print(rightMotorM2->encoder, DEC);
		console.print(F(", Status = "));
		console.print(rightMotorM2->encoderStatus, HEX);
		console.println();
	}

	if (rightMotorM2->speedValid) {
		console.print(F("Right Motor Speed = "));
		console.print(rightMotorM2->speed, DEC);
		console.println();
	}
	
	console.println();
}

/*
	Display the data in a Servo struct
*/
void displayServo (Servo *servo, String servoName) {
	console.println();
	console.print(servo->name);
	console.println(F(" Servo:"));
	console.println();

	console.print(F("Pin = "));
	console.print(servo->pin);
	console.print(F(", msPulse = "));
	console.print(servo->msPulse);
	console.print(F(", Offset = "));
	console.println(servo->offset);

	console.print(F("Home Pos = "));
	console.print(servo->homePos);
	console.print(F(", Angle = "));
	console.println(servo->angle);

	console.print(F("Min Pw = "));
	console.print(servo->minPulse);
	console.print(F(", Max Pw = "));
	console.print(servo->maxPulse);
	console.print(F(", Max Deg = "));
	console.print(servo->maxDegrees);
	console.print(F(", Error = "));
	console.println(servo->error);
	console.println();
}

/*
	Read current data from the RoboClaw 2x5 Motor Controller
*/
uint16_t readRoboClawData (uint8_t address, Motor *leftM1, Motor *rightM2) {
	bool valid;
	uint8_t status;

	//	Error control
	uint16_t errorStatus;
	String errorMsg;

	errorStatus = 0;

	console.println(F("Reading Left Motor Encoder.."));

	leftM1->encoder = roboClaw.ReadEncM1(address, &status, &valid);
	leftM1->encoderStatus = status;
	leftM1->encoderValid = valid;

	console.println(F("Reading Left Motor Speed.."));

	leftM1->speed = roboClaw.ReadSpeedM1(address, &status, &valid);
	leftM1->speedStatus = status;
	leftM1->speedValid = valid;

	console.println(F("Reading Right Motor Encoder.."));

	rightM2->encoder = roboClaw.ReadEncM2(address, &status, &valid);
	rightM2->encoderStatus = status;
	rightM2->encoderValid = valid;

	console.println(F("Reading Right Motor Speed.."));

	rightM2->speed = roboClaw.ReadSpeedM2(address, &status, &valid);
	rightM2->speedStatus = status;
	rightM2->speedValid = valid;

	return errorStatus;
}

/********************************************************************/
/*	Miscellaneous routines 											*/
/********************************************************************/

/*
    Process error conditions
*/
void processError (uint16_t err, String routine, String message) {
	console.print(F("ERROR: "));
	console.print(message);
	console.print(F(", Code = "));
	console.print(err);
	console.print(F(", in routine '"));
	console.print(routine);
	console.println(F("' !"));
}

/*
	Wait for a bit to allow time to read the Console Serial Monitor log
*/
void wait (uint8_t nrSeconds) {
	uint8_t count;

	console.print(F("Waiting"));

	for (count = 0; count < nrSeconds; count++) {
		console.print(F("."));
		delay(1000);
	}

	console.println();
}

/********************************************************************/
/*	RoboClaw 2x5 Motor Controller routines 							*/
/********************************************************************/

/*
	Set motor speeds

	The left and right motor speeds may be different.
*/
void setMotors (uint8_t address, short leftSpd, short rightSpd, Motor *leftM1, Motor *rightM2) {
	bool leftDir, rightDir;

	console.println(F("Setting motor speeds.."));

	leftDir = setMotorSpeed(address, leftSpd, leftM1);
	rightDir = setMotorSpeed(address, rightSpd, rightM2);

	updateMotors(address, leftDir, rightDir, leftM1, rightM2);
}

bool setMotorSpeed (uint8_t address, short spd, Motor *motor) {
	bool direction;

	if (motor->location == Left) {
		//	Set left motor speed
		if (spd >= 0) {
			roboClaw.ForwardM1(address, spd);
			direction = true;
		} else {
			roboClaw.BackwardM1(address, -spd);
			direction = false;
		}
	} else {
		//	Set right motor speed
		if (spd >= 0) {
			roboClaw.ForwardM2(address, spd);
			direction = true;
		} else {
			roboClaw.BackwardM2(address, -spd);
			direction = false;
		}
	}

	return direction;
}

/*
	Update motor data
*/
void updateMotors (uint8_t address, bool leftDir, bool rightDir, Motor *leftM1, Motor *rightM2) {
	bool direction, valid;
	uint8_t speedStatus;
	uint32_t speed;

	console.println(F("Updating motors.."));

	//	Update left motor data
	speed = roboClaw.ReadSpeedM1(address, &speedStatus, &valid);

	leftM1->speed = speed;
	leftM1->speedStatus = speedStatus;
	leftM1->speedValid = valid;
	leftM1->forward = direction;

	//	Update right motor data
	speed = roboClaw.ReadSpeedM2(address, &speedStatus, &valid);

	rightM2->speed = speed;
	rightM2->speedStatus = speedStatus;
	rightM2->speedValid = valid;
	rightM2->forward = direction;
}

/********************************************************************/
/*	Initialization routines 										*/
/********************************************************************/

/*
	Initialize the RoboClaw 2x5 motor controller
*/
void initRoboClaw (uint8_t address, uint16_t bps, Motor *leftM1, Motor *rightM2) {
	console.print(F("Initializing the RoboClaw 2x5 Motor Controller at address "));
	console.print(address, HEX);
	console.print(F(", for "));
	console.print(bps);
	console.println(F(" Bps communication."));

	roboClaw.begin(bps);

	//	Set the RoboClaw motor constants
	roboClaw.SetM1VelocityPID(address, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);
	roboClaw.SetM2VelocityPID(address, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);

	//	For Packet Serial modes
	leftM1->name = ROBOCLAW_MOTOR_LEFT_NAME;
	leftM1->location = Left;
	leftM1->encoder = 0;
	leftM1->encoderStatus = 0;
	leftM1->encoderValid = false;
	leftM1->speed = 0;
	leftM1->speedStatus = 0;
	leftM1->speedValid = false;
	leftM1->forward = true;
	leftM1->distance = 0;
	leftM1->distanceValid = false;		    

	//	For Packet Serial modes
	rightM2->name = ROBOCLAW_MOTOR_RIGHT_NAME;
	rightM2->location = Right;
	rightM2->encoder = 0;
	rightM2->encoderStatus = 0;
	rightM2->encoderValid = false;
	rightM2->speed = 0;
	rightM2->speedStatus = 0;
	rightM2->speedValid = false;
	rightM2->forward = true;
	rightM2->distance = 0;
	rightM2->distanceValid = false;		    
}

/*
	Initialize sensors
*/
void initSensors (void) {
	console.println(F("Initializing Sensors.."));

	console.println(F("     DS1307 Real Time Clock.."));

	//	Check to be sure the RTC is running
//	if (! clock.isrunning()) {
//		console.println(F("The Real Time Clock is NOT running!"));
//		while(1);
//	}
}


/*
	Runs once to initialize everything
*/
void setup (void) {
	uint16_t errorStatus;

	//  Start up the Wire library
	Wire.begin();

	//  Initialize the console port
	console.begin(9600);

	console.println();
	console.print(F("W.A.L.T.E.R. 2.0 Motor and Controller Test, version "));
	console.print(BUILD_VERSION);
	console.print(F(" on "));
	console.print(BUILD_DATE);
	console.print(F(" for the "));
	console.print(BUILD_BOARD);
	console.println(F(" board"));

	console.println();

	console.println(F("Initializing Digital Pins.."));

	//  Initialize the LED pin as an output.
	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED, LOW);

	console.println();

	if (ROBOCLAW_CONTROLLERS > 0) {
		//	Initialize the RoboClaw 2x5 motor controller port
		initRoboClaw(roboClawAddress1, 38400, &leftMotorM1, &rightMotorM2);

		//	Let's see if we can move
		setMotors(roboClawAddress1, 100, 100, &leftMotorM1, &rightMotorM2);
	}
}

/*
	Runs forever
*/
void loop (void) {
	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

	//	The current date and time from the DS1307 real time clock
	DateTime now = clock.now();

	//	Display related variables
	bool amTime, pitchRollValid = false, headingValid = false;
	uint8_t displayNr = 0, count = 0;
	uint8_t readingNr = 0, areaClosestReadingPING = 0, areaClosestReadingIR = 0;
	uint8_t areaFarthestReadingPING = 0, areaFarthestReadingIR = 0;
	uint8_t currentHour = now.hour(), nrDisplays = 0;
	uint16_t displayInt;

	uint8_t roboClawStatus;
	bool roboClawValid;

	uint8_t analogPin = 0;
	uint8_t digitalPin = 0;

	/*
		Code starts here
	*/

	// Pulse the heartbeat LED
	pulseDigital(HEARTBEAT_LED, 500);

	currentMinute = now.minute();

	/*
		This is code that only runs one time, to initialize
			special cases.
	*/
	if (firstLoop) {
		lastMinute = currentMinute;

		firstLoop = false;
	}

	if (ROBOCLAW_CONTROLLERS > 0) {
		/*
			Read the RoboClaw 2x5 motor data
		*/
		console.println(F("Reading RoboClaw.."));

		errorStatus = readRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);

		if (errorStatus != 0) {
			processError(errorStatus, "readRoboClawData", "Unhandled problem");
		} else {
			displayRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);
		}
	}

	if (ROBOCLAW_CONTROLLERS > 0) {
		/*
			Read the RoboClaw 2x5 motor data
		*/
		console.println(F("Reading RoboClaw.."));

		errorStatus = readRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);

		if (errorStatus != 0) {
			processError(errorStatus, "readRoboClawData", "Unhandled problem");
		} else {
			displayRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);
		}
	}

	console.println();

	//	Count the minutes
	if (currentMinute != lastMinute) {
		minuteCount += 1;
		lastMinute = currentMinute;
	}

	wait(WAIT_DELAY_SECONDS);

	console.println();
}
