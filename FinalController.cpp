// UDP Joystick Final Code
#include "Particle.h"
#include <iostream>
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);
 
// Joystick setup
int vertPin = A1;
int horizPin = A0;
int xVal;
int yVal;
int deadZone = 10; // To stop jittering from varying joystick values
char v[5] = {' '};


// UDP Port used for two way communication
unsigned int localPort = 8888;
bool wasConnected = false;


// An UDP instance to let us send and receive packets over UDP
UDP Udp;


void setup() {
  // Begin UDP/Serial
  Udp.begin(localPort);
  Serial.begin(9600);
}


void loop() {
  //Read joystick input
  xVal = analogRead(vertPin);
  yVal = analogRead(horizPin);

  //if we fall within the deadzone, set it back to default value for use in carFinal
  if (xVal < (2042 + deadZone) && xVal > (2042 - deadZone)) { xVal = 2042; }
  if (yVal < (2100 + deadZone) && yVal > (2100 - deadZone)) { yVal = 2102; }
  

  // Store sender ip and port
  IPAddress ipAddress(10, 130, 139, 145);


  // Translate xVal/yVal into chars and send
  Udp.beginPacket(ipAddress, localPort);
 
  //sprintf converts integers in xVal to characters so they can be placed
  //in the array v. %d just tells it to expect numbers to convert.
  sprintf(v, "%d", xVal); 
  Udp.write(v);
  Udp.write(' ');
  Serial.print(v);
  Serial.print(' ');


  for (int i = 0; i < 5; ++i) { v[i] = ' '; } // Clear list to white spaces

  //do the same thing above over again
  sprintf(v, "%d", yVal);
  Serial.print(v);
  Serial.print('\n');
  Udp.write(v);
  Udp.endPacket();


  delay(50);
  //We slightly slow the sending of packets so that unexpected behavior doesn't occur
}


