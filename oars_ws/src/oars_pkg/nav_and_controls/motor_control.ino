/*  Sail & rudder position receiver
 *  
 *  Receive sail & rudder position over serial and control motors using PWM.
 *  Intended for communication via XBee modules, but generally works for any
 *  properly configured serial connection.
 *  
 *  Messages should be formatted as (SAIL,RUD)
 *  
 *  Author: Jane Sieving
 *  
*/

#include <Servo.h>

const int sailPin = 10;  // SAIL
const int rudPin = 9;   // RUDDER

Servo sail;
Servo rudder;
int sailPos; 
int rudPos;
bool started = false;   // True: Message is started
bool mid = false;      // True: Second half of message
bool ended  = false;  // True: Message is finished 
char incomingByte ;  // Variable to store the incoming byte
char msgS[5];       // Sail message: PWM value
char msgR[5];      // Rudder message: PWM value
byte indexS;      // Index in sail message
byte indexR;     // Index in rudder message

                     // Leaving these values pretty unrestricted until this is calibrated
int SAIL_MAX = 360  // 100 
int SAIL_MIN = -1  // 20
int RUD_MAX = 360 // 120
int RUD_MIN = -1 // 60

void setup() {
  //  Start serial communication
  Serial.begin(9600); // Baud rate must be the same as on xBee module
  sail.attach(9);
  rudder.attach(10);
  sailPos = 90;
  rudPos = 90;
  sail.write(sailPos);
  rudder.write(rudPos);
}

// Gradually move sail to a given position
void moveSto(int pos) {
  do {
    if (sailPos < pos && sailPos < SAIL_MAX) {sailPos++;}
    else if (sailPos > pos && sailPos > SAIL_MIN) {sailPos--;}
    sail.write(sailPos);
    delay(10);
  } while (sailPos != pos && sailPos > SAIL_MIN && sailPos < SAIL_MAX);
}

// Gradually move rudder to a given position
void moveRto(int pos) {
  do {
    if (rudPos < pos && rudPos < RUD_MAX) {rudPos++;}
    else if (rudPos > pos && rudPos > RUD_MIN) {rudPos--;}
    rudder.write(rudPos);
    delay(10);
  } while (rudPos != pos && rudPos > RUD_MIN && rudPos < RUD_MAX);
}

void loop() {  
  // Read an interpret incoming messages
  while (Serial.available() > 0){
    // Read the incoming byte
    incomingByte = Serial.read();    
    
    // Start the message when the '(' symbol is received
    if(incomingByte == '(')
    {
      started = true;
      indexS = 0;
      indexR = 0;
      msgS[indexS] = '\0'; // Throw away any incomplete message
      msgR[indexR] = '\0';
    }
    //End the message when the ')' symbol is received
    else if(incomingByte == ')')
    {
      ended = true;
      break; // Done reading
    }
    else if (incomingByte == ','){
      mid = true;
    }
    //Read the value
    else
    {
      if (!mid) {
        if(indexS < 5) // Make sure there is room
        {
          msgS[indexS] = incomingByte; // Add char to array
          indexS++;
          msgS[indexS] = '\0'; // Add NULL to end
        }
      }
      else {
        if(indexR < 5) // Make sure there is room
        {
          msgR[indexR] = incomingByte; // Add char to array
          indexR++;
          msgR[indexR] = '\0'; // Add NULL to end
        }
      }
    }
  }

  // Reset values after complete message
  if(started && ended)
  {
    int valueS = atoi(msgS);
    int valueR = atoi(msgR);
    moveRto(valueR);
    moveSto(valueS);
    Serial.print("Sail: ");
    Serial.println(sailPos); // For debugging
    Serial.print("Rudder: ");
    Serial.println(rudPos); // For debugging
    indexS = 0;
    indexR = 0;
    msgS[indexS] = '\0';
    msgR[indexR] = '\0';
    started = false;
    mid = false;
    ended = false;
  }
}