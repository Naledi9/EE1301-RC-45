// Standard library
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// UDP Setup
UDP Udp;
unsigned int port = 8888; //no particular reason for this port, it's just the documentation standard
bool wasConnected = false;

//Actuator Variables
int IN1 = D2; //Forward
int IN2 = D3; //Reverse
int ServoPIN = A5; //servo
Servo steeringServo;
int xVal = 90; //turner
int yVal = 2048; //mover

//Sensor data
int dataTempV = A2;
int lightPin = A1;
int ADCreading; // This will be used for both temp and light sensor
double tempC = 0.0;
double tempF; //Relates to cloud variable TempCloudF
int lightLevel;
int lightPercent; //Relates to cloud variable lightCloud

// Website control variables
bool websiteCtrl = false;
int forward = 2; // 1 is backwards, 2 is stop, and 3 is forwards
int LR = 1; // 1 is no direction, 2 is left, and 3 is right

// Cloud function declarations
int setDir(String input);

void setup() {
 // UDP
 Udp.begin(port);
 
 // Attach servo and motors
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 steeringServo.attach(ServoPIN);
 steeringServo.write(90); // 90 degrees is centered

 // Start temp sensor
 pinMode(dataTempV, INPUT);

 // Cloud variables
 Particle.variable("TempCloudF", tempF);
 Particle.variable("lightCloud", lightPercent);

 // Cloud function
 Particle.function("c_setDir", setDir);

 // Serial for debugging
 Serial.begin(9600);
}

void loop() {
 // IP Printing
 static bool debug = true;
 if (debug) {
  delay(20000);
  Serial.print("localIP=");
  Serial.print(WiFi.localIP().toString().c_str());
  Serial.print('\n');
  debug = false;
 }

 // Receive UDP Packet
 if (Udp.parsePacket() > 0) {
  char c; // Current char
   char v[5] = {' '}; // Buffer, when we reach this, we know we're moving on to the turner values
   int i = 0;

   while(Udp.available()) //as long as there is data to be collected, continue
    {
    c = Udp.read(); //take next character in the data packet
    if (c == ' ') { //if we have reached the buffer, we define xVal, and begin refilling the array for yVal.
     i = 0;
     xVal = ((atoi(v) - 2048) / 68.0) + 90;
     for (int i = 0; i < 5; ++i) { v[i] = ' '; } //clear array for future use
    } else {
      v[i] = c; //fill array
    }
    ++i;
   }
  yVal = atoi(v);
 }

 // Is the controller in the default position?
 bool defaultPosition = (xVal == 90 || xVal == 89) && (yVal >= 1800 && yVal <= 3000); 

 // Switch back to controller when it is used
 if(!defaultPosition || !websiteCtrl) {
  websiteCtrl = false;
  steeringServo.write(xVal);
  Serial.print(xVal); //debug
  Serial.print(' ');
  Serial.print(yVal);
  Serial.print('\n');

 // This section fires the motors when yVal (mover reaches certain thresholds)
 if (yVal > 3000) {
  //Reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
 } else if (yVal < 1800) {
  //Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
 } else {
  //stop movement
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
 }
}

// Website controlling
if (defaultPosition && websiteCtrl) {
   if(forward == 3) {
   //forward options
   if(LR == 1) {
	//pure forward
	//Set servo to centered
	steeringServo.write(90); // 90 = center

	// Move forward
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
   }
   else if(LR == 2) {
	//left forward
	//place servo in left position
	steeringServo.write(60);
	//move forward
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
   }
   else if(LR == 3) {
	//right forward
	//place servo in right position
	steeringServo.write(120);
	//move forward
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
   }
 }
 else if(forward == 1) {
  //reverse options
  if(LR == 1) {
	//pure reverse
	//servo to center
	steeringServo.write(90);
	// Reverse
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
  }
  else if(LR == 2) {
	//Left reverse
	//servo to left position
	steeringServo.write(60);
	// Reverse
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
  }
  else if(LR == 3) {
	//Right reverse
	//servo to right position
	steeringServo.write(120);
	// Reverse
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
  }
 }
 else if(forward == 2) {
  //stop, don't change servo direction because it doesn't really matter.
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, LOW);
 }
}

 // Send sensor data to website
 ADCreading = analogRead(dataTempV);
 lightLevel = analogRead(lightPin);
 tempC = (ADCreading - 620) * 0.0806;
 tempF = (tempC*(9.0/5))+32;
 lightPercent = lightLevel/4096.0*100;

 // End of loop
}

// Change the direction of the car over the cloud
// Backward is 1, stopped is 2, forward is 3
// Centered is 1, left is 2, right is 3
int setDir(String input) {
  websiteCtrl = true;
  Serial.println(input); //debug
  if(input == "F") {
	//Pure forward
	forward = 3;
	LR = 1;
  }
  else if(input == "LF") {
	//Left forward
	forward = 3;
	LR = 2;
  }
  else if(input == "RF") {
	//Right forward
	forward = 3;
	LR = 3;
  }
  else if(input == "B") {
	//Pure back
	forward = 1;
	LR = 1;
  }
  else if(input == "LB") {
	//Left back
	forward = 1;
	LR = 2;
  }
  else if(input == "RB") {
	//Right back
	forward = 1;
	LR = 3;
  }
  else if(input == "S") {
	//full stop
	forward = 2;
  }
  else {
	//error
	return -1;
  }
  return 1;
}

