/****************************************************************
*                    Arduino MD25 example code                  *
*                   MD25 running in Serial1 mode                 *
*                                                               *
*                     by James Henderson 2012                   *
*****************************************************************/

#include <SoftwareSerial.h>
#include "KerbalSimpit.h"

// Values of 0 being sent using Serial1.write() have to be cast as a byte to stop them being misinterpereted as NULL
// This is a bug with arduino 1
#define CMD                 (byte)0x00              //  MD25 command byte of 0

#define WRITESP1            0x31                    // Byte writes speed to motor 1
#define WRITEACCEL          0x33                    // Byte writes a value of acceleration
#define RESETREG            0x35                    // Byte resets encoders
#define SETMODE             0x34                    // Byte sets mode
#define READIVS             0x2C                    // Byte reads motor currents and battery voltage        
#define READENCS            0x25                    // Byte reads both encoders
#define GET_VER             0x29

#define LCD_RX              0x02                    // RX and TX pins used for Serial03 Serial1 port
#define LCD_TX              0x03
#define Serial_HIDE_CUR      0x04
#define Serial_CLEAR         0x0C
#define Serial_SET_CUR       0x02

//SoftwareSerial1 Serial = SoftwareSerial1(LCD_RX, LCD_TX);          // Define the Serial1 port for the Serial
KerbalSimpit mySimpit(Serial);
int16_t pitch;
long encValue = 0;
byte softwareRev = 0;
// This boolean tracks the desired LED state.
bool state = false;

// A timestamp of the last time we sent an echo packet.
unsigned long lastSent = 0;
// How often to send echo packets (in ms)
unsigned int sendInterval = 1000;



void setup(){
  Serial1.begin(38400);
  Serial.begin(115200);
  
  Serial1.write(CMD);                                            // Set MD25 accelleration value
  Serial1.write(WRITEACCEL);
  Serial1.write(10);
  delayMicroseconds(10);                                        // Wait for this to be processed
  Serial1.write(CMD);                                            // Reset the encoder registers to 0
  Serial1.write(RESETREG);         
  Serial1.write(CMD);                                            // Set mode to 2, Both motors controlled by writing to speed 1
  Serial1.write(SETMODE);
  Serial1.write(2);    
  
  Serial1.write(CMD);                                            // Get software version of MD25
  Serial1.write(GET_VER);
  while(Serial1.available() < 1){}                               // Wait for byte to become available         
  
  // Set up the built in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    delay(100);
  }

    // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);
  // Display a message in KSP to indicate handshaking is complete.
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  mySimpit.inboundHandler(messageHandler);
  
  softwareRev = Serial1.read();  
  
//  Serial.write(Serial_CLEAR);                                     // Clear the Serial screen
 // Serial.write(Serial_HIDE_CUR);                                  // Hide the Serial cursor
}

void loop(){ 
  
  /*while(encValue < 3000){               // While encoder 1 value less than 3000 move forward
    Serial1.write(CMD);                  // Set motors to drive forward at full speed
    Serial1.write(WRITESP1);
    Serial1.write(150);
    encValue = readEncoder();
    readVolts();
  }
  delay(100);
  while(encValue > 100){
    Serial1.write(CMD);                  // Drive motors reverse at full speed
    Serial1.write(WRITESP1);
    Serial1.write(100);
    encValue = readEncoder();
    readVolts();
  }*/
//  delay(100);
  encValue = readEncoder();
  rotationMessage rot_msg;

 unsigned long now = millis();
  // If we've waited long enough since the last message, send a new one.
  if (now - lastSent >= sendInterval) {
    // If the last message was "high", send "low"
    // and vice-versa.
    if (state) {
      mySimpit.send(ECHO_REQ_MESSAGE, "low", 4);
    } else {
      mySimpit.send(ECHO_REQ_MESSAGE, "high", 5);
    }
    // Update when we last sent a message.
    lastSent = now;
    // Update the state variable to match the message we just sent.
    state = !state;
  }
  // Call the library update() function to check for new messages.
  mySimpit.update();

  
  
  pitch = map(encValue, 0, 1023, INT16_MIN, INT16_MAX);
  //rot_msg.setPitch(pitch);
  //mySimpit.send(ROTATION_MESSAGE, rot_msg);
} 

long readEncoder(){                        // Function to read and display the value of both encoders, returns value of first encoder
  long result1 = 0; 
  long result2 = 0;
  Serial1.write(CMD);
  Serial1.write(READENCS);
  while(Serial1.available() < 8){}          // Wait for 8 bytes, first 4 encoder 1 values second 4 encoder 2 values 
  result1 = Serial1.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += Serial1.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += Serial1.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += Serial1.read();               // Fourth byte for encoder 1, LL
  result2 = Serial1.read();
  result2 <<= 8;
  result2 += Serial1.read();
  result2 <<= 8;
  result2 += Serial1.read();
  result2 <<= 8;
  result2 += Serial1.read();
/*
  Serial.write(Serial_SET_CUR);              // Set the Serial cursor position
  Serial.write(21);
  Serial.print("Encoder 1:");               // Displays data to the Serial screen
  Serial.print(result1,DEC);
  Serial.print("Pitch:");
  Serial.print(pitch,DEC);
  Serial.print("\n");                        // Print a blank space to clear any unwanted characters that are leftover on the Serial display
  
  delay(5);                                // Delay for Serial to process data
  
  Serial.write(Serial_SET_CUR);
  Serial.write(41); 
  Serial.print("Encoder 2:");
  Serial.print(result2,DEC);
  Serial.print(" ");*/
  return result1;                                   
}
  
void readVolts(){                                                 // Function reads current for both motors and battery voltage
  byte batteryVolts, mot1_current, mot2_current = 0;
  Serial1.write(CMD);
  Serial1.write(READIVS);                                          // Send byte to readbattery voltage and motor currents
  while(Serial1.available() < 3){}                                 // Wait for the 3 bytes to become available then get them
  batteryVolts = Serial1.read();
  mot1_current = Serial1.read();
  mot2_current = Serial1.read();
/*
  Serial.write(Serial_SET_CUR);
  Serial.write(61);
  Serial.print("Mot1 I:");
  Serial.print(mot1_current,DEC);
  Serial.print(" Mot2 I:");
  Serial.print(mot2_current,DEC);
  Serial.print("\n"); 
  
  delay(5);
  
  Serial.write(Serial_SET_CUR);
  Serial.write(1);
  Serial.print("Rev:");
  Serial.print(softwareRev, DEC);
  Serial.print(" ");
  Serial.print("Volts:");
  Serial.print(batteryVolts/10,DEC);                               // Seperate whole number and descimal place of battery voltage and display
  Serial.print(".");  
  Serial.print(batteryVolts%10,DEC);
  Serial.print("\n");   */
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  // We are only interested in echo response packets.
  if (messageType == ECHO_RESP_MESSAGE) {
    // The message payload will be either "low" or "high".
    // We use the strcmp function to check what the string payload
    // is, and set the LED status accordingly.
    if (strcmp((char*) msg, "low")) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}
