/* KerbalSimpitAltitudeTrigger
   A demonstration of subscribing to telemetry data from the game.
   Subscribes to the altitude channel, and turns the pin 13 LED
   on when the sea level altitude > 500m.

   Peter Hardy <peter@hardy.dropbear.id.au>
*/

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

//***** Druck sensor
#include <Q2HX711.h>

const byte MPS_OUT_pin = 20; // OUT data pin
const byte MPS_SCK_pin = 21; // clock data pin
int avg_size = 1; // #pts to average over

Q2HX711 MPS20N0040D(MPS_OUT_pin, MPS_SCK_pin); // start comm with the HX710B

//******************


typedef struct encoderValues{
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
void setup() {
  

  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  init_simpit();
  init_encoder();  
  
  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

}

long read_pressure() {
    float avg_val = 0.0; // variable for averaging
  for (int ii=0;ii<avg_size;ii++){
    avg_val += MPS20N0040D.read(); // add multiple ADC readings
    delay(50); // delay between readings
  }
  avg_val /= avg_size;
  long res =  round(avg_val) - 9007500;
  return  res;
}

void init_simpit() {
// Open the serial connection.
  Serial3.begin(115200);

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    delay(100);
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
}

void init_encoder() {
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

  
}

long mapPressure(long x, long in_min, long in_max) {
  // Convert the range into double to handle larger intermediate values
  double diff = x - in_min;
  double range = in_max - in_min;
  
  double scaled = diff / range; // Calculate the scaling factor
  double result = scaled * INT16_MAX; // Apply the scaling factor to the output range
  
  return (long)result; // Convert the double result back to long
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
   long roll = mapPressure(pressure, 0, 10962926);
  rot_msg.setPitch(pitch);
  rot_msg.setYaw(yaw);
  rot_msg.setRoll(roll);
  mySimpit.send(ROTATION_MESSAGE, rot_msg);
  mySimpit.update();
  String text = " y " + String(yaw) + " x " + String(pitch) +" r " + String(roll);
  mySimpit.printToKSP(text, NO_HEADER);
  
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
