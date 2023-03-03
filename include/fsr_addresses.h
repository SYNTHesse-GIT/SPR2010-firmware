// addresses for registers
// using namespace ControlTableItem;
enum ControlTableItemAddr: uint16_t {
  // ROM items
  // these are the ones from "slave.h"
  ADDR_ITEM_MODEL_NUMBER      = 0,      // same as in "slave.h"
  ADDR_ITEM_FIRMWARE_VERSION  = 6,      // same as in "slave.h"
  ADDR_ITEM_ID                = 7,      // same as in "slave.h"
  ADDR_ITEM_PROTOCOL_VER      = 9,      // same as in "slave.h"

  // new
  ADDR_ITEM_MODEL_INFO        = 2,      // 4 bytes
  ADDR_ITEM_BAUD_RATE         = 8,      // 1 byte
  ADDR_ITEM_RETURN_DELAY_TIME = 13,     // 1 byte

  ADDR_ITEM_VOLTAGE_SAMPLES   = 14,     // how many reading samples for voltage
  ADDR_ITEM_VOLTAGE_TH4       = 16,     // voltage threshhold led4 in 0.01V units
  ADDR_ITEM_VOLTAGE_TH3       = 18,     // voltage threshhold led3 in 0.01V units
  ADDR_ITEM_VOLTAGE_TH2       = 20,     // voltage threshhold led2 in 0.01V units
  ADDR_ITEM_VOLTAGE_TH1       = 22,     // voltage threshhold led1 in 0.01V units
  ADDR_ITEM_VOLTAGE_TH1B      = 24,     // voltage threshhold led1 blink in 0.01V units
  ADDR_ITEM_VOLTAGE_HYST      = 26,     // voltage threshhold hysteresis in 0.01V units
  ADDR_ITEM_VOLTAGE_FACTOR    = 28,     // voltage calculation factor
  ADDR_ITEM_CURRENT_FACTOR    = 30,     // current calculation factor

  // RAM items
  ADDR_ITEM_VOLTAGE           = 70,     // battery voltage in 0.1V units
  ADDR_ITEM_CURRENT           = 72,     // current consumption in mA
  ADDR_ITEM_BATTERY_PERC      = 74,     // coded battery percentage

  ADDR_ITEM_LOOP_RATE         = 98,     // 2 bytes current loop rate in Hz

//   // specific for FSR
//   ADDR_ITEM_LPF_FSR           = 14,     // 1 byte low pass filter scaler for FSR
//   ADDR_ITEM_LPF_VOL           = 15,     // 1 byte low pass filter scaler for Voltage

//   // calibration
//   ADDR_ITEM_FL_MUL            = 20,     // multiplier for FL
//   ADDR_ITEM_FL_DIV            = 22,     // divisor for FL
//   ADDR_ITEM_FR_MUL            = 24,     // multiplier for FR
//   ADDR_ITEM_FR_DIV            = 26,     // divisor for FR
//   ADDR_ITEM_BL_MUL            = 28,     // multiplier for BL
//   ADDR_ITEM_BL_DIV            = 30,     // divisor for BL
//   ADDR_ITEM_BR_MUL            = 32,     // multiplier for BR
//   ADDR_ITEM_BR_DIV            = 34,     // divisor for BR

//   ADDR_ITEM_LAST_ROM          = 64,     // used for handling the save / retrieve

//   // RAM items FSR
//   ADDR_ITEM_ENABLE            = 64,     // 1 byte enable device

//   // raw readings
//   ADDR_ITEM_RAW_FL            = 70,     // 2 bytes raw reading front left
//   ADDR_ITEM_RAW_FR            = 72,     // 2 bytes raw reading front right
//   ADDR_ITEM_RAW_BL            = 74,     // 2 bytes raw reading back left
//   ADDR_ITEM_RAW_BR            = 76,     // 2 bytes raw reading back right

//   // LPF items FSR
//   ADDR_ITEM_LPF_FL            = 78,     // 2 bytes LPF reading front left
//   ADDR_ITEM_LPF_FR            = 80,     // 2 bytes LPF reading front right
//   ADDR_ITEM_LPF_BL            = 82,     // 2 bytes LPF reading back left
//   ADDR_ITEM_LPF_BR            = 84,     // 2 bytes LPF reading back right

//   // voltage
//   ADDR_ITEM_VOLTAGE           = 85,     // 1 byte raw voltage in 0.1V
//   ADDR_ITEM_CURRENT           = 86,     // 1 byte raw current in 0.01A

//   // LPF voltage
//   ADDR_ITEM_LPF_VOLTAGE       = 87,     // 1 byte LPF voltage in 0.1V
//   ADDR_ITEM_LPF_CURRENT       = 88,     // 1 byte LPF current in 0.01A



//   // calibrated items FSR
//   ADDR_ITEM_CAL_FL            = 100,     // 2 bytes calibrated LPF front left
//   ADDR_ITEM_CAL_FR            = 102,     // 2 bytes calibrated LPF front left
//   ADDR_ITEM_CAL_BL            = 104,     // 2 bytes calibrated LPF front left
//   ADDR_ITEM_CAL_BR            = 106      // 2 bytes calibrated LPF front left
  
//  ADDR_ITEM_CENTER_X          = 78,     // processed center X
//  ADDR_ITEM_CENTER_Y          = 80,     // processed center Y
//  ADDR_ITEM_CENTER_F          = 82,     // processed center Force
//  // bumpers
//  ADDR_ITEM_BUMPERS           = 84,     // bumpers 6 bits: [0, 0, BL, BC, BR, FL, FC, FR]
};
