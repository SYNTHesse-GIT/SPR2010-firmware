#include <Dynamixel2Arduino.h>
#include "fsr_pins.h"
#include "fsr_addresses.h"
#include "fsr_registers.h"

#define DXL_DIR_PIN PA8        // Dynamixel port direction pin
#define DXL_PORT    Serial1     // port for communication

const uint16_t  FSR_ADC_RESOLUTION      = 4096;
// Rev C
// const uint16_t  FSR_VOLTAGE_FACTOR      = 1425;  // (47+10)/10 x 2.5 x 100 [0.01V units]
// const uint16_t  FSR_CURRENT_FACTOR      = 2500; // 2.5V x 1000 [mA]

// Dev D
const uint16_t  FSR_VOLTAGE_FACTOR      = 1881;  // (47+10)/10 x 3.3 x 100 [0.01V units]
// Vout = 0.01 x Vsense x Rout
// Vout = 0.01 x Rsense x I x Rout
// Vout = 0.01 x 10e-3(10mohm) x I x 10e3(10kohm)
// Vout = I
// Vout is directly equal with the current going through R sense
// (ex. if current is 1.5A the Vout will be 1.5V)
const uint16_t  FSR_CURRENT_FACTOR      = 3300; // 3.3V x 1000 [mA]

enum {
    STATE4 = 87,  // (100 + 75) / 2; will show all 4 LEDs on
    STATE3 = 62,  // (75 + 50) / 2 ; will show 3 LEDs on
    STATE2 = 37,  // (50 + 25) / 2 ; will show 2 LEDs on
    STATE1 = 12,  // (25 + 0) / 2  ; will show 1 LED on solid
    STATE1B = 6,  //               ; will show 1 LED blinking, buzzer 1Hz 50%
    STATESD = 0   //               ; will show 1 LED blinking 1Hz, 10%, power cutoff
};

class FSRDevice : public DYNAMIXEL::Slave
{
private:
    ROM     rom;
    RAM     ram = {};
    // DYNAMIXEL::SerialPortHandler dxl_port;
    uint8_t battery_state_ = STATE4;

    uint32_t getBaudrateValueFromIndex(uint8_t baud_index);
    uint8_t  getErrorDataRange();

public:
    FSRDevice(DYNAMIXEL::SerialPortHandler &dxl_port);
    void setupPins(void);
    void readConfigFromROM(void);
    void addControlItems(void);
    void startDXL();

    bool updateVoltage(void);
    void updateBatteryState(void);
    void displayBatteryState(void);

    void processFSR(void);

    void processStatistics(void);

    void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg);
    void read_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg);
};
