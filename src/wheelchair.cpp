/* KerbalSimpitAltitudeTrigger
   A demonstration of subscribing to telemetry data from the game.
   Subscribes to the altitude channel, and turns the pin 13 LED
   on when the sea level altitude > 500m.

   Peter Hardy <peter@hardy.dropbear.id.au>
*/
#include <Arduino.h>
#include "KerbalSimpit.h"
#include <LiquidCrystal_I2C.h> //library via Include Library > Manage Library > 


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

//***** Druck sensor
#include <Q2HX711.h>


uint8_t simpit_init = false;
uint8_t pressure_init = false;
uint8_t motor_init = false;
char lcd_status[255];


LiquidCrystal_I2C lcd(0x27,16,2); 

void prg_status() {
  lcd.setCursor(0,1);
  sprintf(lcd_status, "S:%d P:%d M:%d", simpit_init, pressure_init, motor_init);
  lcd.print(lcd_status );
}


const byte MPS_OUT_pin = 6; // OUT data pin
const byte MPS_SCK_pin = 7; // clock data pin

// Define the actual input value that should map to zero
long zeroInput = 0; // Update this value based on your calibration
void messageHandler(byte messageType, byte msg[], byte msgSize);
void init_pressure();
void init_encoder();

Q2HX711 MPS20N0040D(MPS_OUT_pin, MPS_SCK_pin); // start comm with the HX710B

//******************

struct encoderValues{
  long encoder1;
  long encoder2;
} ;

encoderValues encValues;
// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial3);
 int16_t pitch, yaw, roll;
 int reading_pitch=0;
 byte softwareRev = 0;



void init_pressure() {
  float avg_val = 0.0; // variable for averaging
  int avg_size = 20;
  char sPressure[300];
  lcd.setCursor(0,0); 
  lcd.print("Init pressure");

  for (int ii=0;ii<avg_size;ii++){
    avg_val += MPS20N0040D.read(); // add multiple ADC readings
    delay(100); // delay between readings
    lcd.setCursor(0,0); 
    sprintf(sPressure, "P:%f", avg_val);
    lcd.print(sPressure);
  }
  avg_val /= avg_size;
  zeroInput = long(avg_val);
  lcd.setCursor(0,0); 
  sprintf(sPressure, "P:%d", zeroInput);
  lcd.print(sPressure);

  pressure_init = true;
  prg_status();

  
}

void init_lcd() {
  lcd.init();
  lcd.setCursor(0,0);
  lcd.print("Lcd init");
  lcd.backlight();
  //lcd.autoscroll();
}

void init_encoder() {
  lcd.setCursor(0,0); 
  lcd.print("Init encoder");

  Serial1.begin(38400);
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
   softwareRev = Serial1.read();  

  motor_init = true;
  prg_status();

}




void init_simpit() {
// Open the serial connection.
  Serial3.begin(115200);

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    lcd.setCursor(0,0);
    lcd.print("Simpit init...");
  }
  // Display a message in KSP to indicate handshaking is complete.
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  mySimpit.inboundHandler(messageHandler);
  // Send a message to the plugin registering for the Altitude channel.
  // The plugin will now regularly send Altitude messages while the
  // flight scene is active in-game.
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  simpit_init = true;
  prg_status();
}


long read_pressure() {
    float avg_val = 0.0; // variable for averaging
    int avg_size = 1;
  for (int ii=0;ii<avg_size;ii++){
    avg_val += MPS20N0040D.read();  // add multiple ADC readings
    delay(50); // delay between readings
  }
  avg_val /= avg_size;
  return  long(avg_val);
}

void setup() {
  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  init_lcd();
  prg_status();
  init_simpit();
  init_encoder();  
  init_pressure();
  
  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

}

// Function to map pressure readings to a 16-bit signed integer range, centered around 0
long mapPressure(long x, long in_min, long in_max) {
  // Adjust x by shifting it to center on zeroInput
  long adjustedX = x - zeroInput;
  
  // Now map from adjustedX to the output range
  double scaled = (double)adjustedX / (in_max - in_min); // Normalize adjustedX
  double result = scaled * (INT16_MAX - INT16_MIN); // Scale normalized value to full int16 range

  return (long)result; // Convert the double result back to long
}


struct encoderValues readEncoder(){                        // Function to read and display the value of both encoders, returns value of first encoder
  long result1 = 0; 
  long result2 = 0;
  encoderValues encoders;
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
  encoders.encoder1 = result1;
  encoders.encoder2 = result2;
  return encoders;                                   
}


void loop() {


rotationMessage rot_msg;
rotationMessage yaw_msg;
rotationMessage roll_msg;
  encValues = readEncoder();
  long pressure = read_pressure();
  //reading_pitch = reading_pitch+1;
  // Check for new serial messages.
  pitch = map(encValues.encoder1, 0, 1023, INT16_MIN, INT16_MAX);
  yaw = map(encValues.encoder2, 0, 1023, INT16_MIN, INT16_MAX);
   long roll = mapPressure(pressure, 0, 13962926);
  if (roll > -600 && roll < 600)
    roll = 0;
   if (roll < -35000)
    roll = -32000;
  rot_msg.setPitch(pitch);
  rot_msg.setYaw(yaw);
  rot_msg.setRoll(roll);
  mySimpit.send(ROTATION_MESSAGE, rot_msg);
  mySimpit.update();
  //String text = " y " + String(yaw) + " x " + String(pitch) +" r " + String(roll);
  String text = " x " + String(pitch) +" r " + String(roll);
  lcd.setCursor(0,0);
  lcd.print(text);
  
  mySimpit.printToKSP(text, NO_HEADER);
  
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch(messageType) {
  case ALTITUDE_MESSAGE:
    // Checking if the message is the size we expect is a very basic
    // way to confirm if the message was received properly.
    if (msgSize == sizeof(altitudeMessage)) {
      // Create a new Altitude struct
      altitudeMessage myAltitude;
      // Convert the message we received to an Altitude struct.
      myAltitude = parseMessage<altitudeMessage>(msg);
      // Turn the LED on if the vessel is higher than 500 metres
      // above sea level. Otherwise turn it off.
      if (myAltitude.sealevel > 1500) {
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    break;
  }
}