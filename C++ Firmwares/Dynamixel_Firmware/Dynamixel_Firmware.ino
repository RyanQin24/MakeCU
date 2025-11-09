#include <Dynamixel2Arduino.h>
#include <actuator.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

//safety


void setup() {
    // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
  dxl.torqueOn(DXL_ID1);

  dxl.ping(DXL_ID2);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
  dxl.torqueOn(DXL_ID2);

}

void loop() {
  if(DEBUG_SERIAL.available()){
      String message = "";
      message = DEBUG_SERIAL.readStringUntil('>'); // reads until '>' is found
      if(message.indexOf(",") > 0){
        int del_ind = message.indexOf(',');
        float velocity1 = message.substring(0,del_ind).toFloat();
        float velocity2 = message.substring(del_ind + 1, message.length()).toFloat();
        dxl.setGoalVelocity(DXL_ID1, velocity1);
        dxl.setGoalVelocity(DXL_ID2, velocity2);
      }
  }

}
