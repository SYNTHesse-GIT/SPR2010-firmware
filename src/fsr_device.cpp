#include <EEPROM.h>
#include "fsr_device.h"

// #define FORCE_RELOAD_ROM

// convenience macros to turn on/off LEDs
#define LEDON(n)    digitalWrite(LED##n, LOW)  // LEDs are invers logic
#define LEDOFF(n)   digitalWrite(LED##n, HIGH)
#define noCutoff    digitalWrite(CUTOFF, HIGH); // power the external DXL bus
#define cutoff      digitalWrite(CUTOFF, LOW);  // cutoff the external DXL bus


/**
 * @brief Default ROM content to be used when staring the firmware first time
 * or when updating the firmware.
 * NOTE:
 * When updating the firmware the content of the ROM will be deleted and
 * reverted to the defaults bellow.
 */
ROM defaultROM = {
    DEF_MODEL_NUMBER,
    DEF_MODEL_INFO,
    DEF_FIRMWARE_VERSION,
    100,                        // ID
    3,                          // baudrate
    2,                          // protocol type
    0,                          // return delay time
    10,                         // voltage samples
    1200,                       // volt th 4 = 12.0V
    1140,                       // volt th 3 = 11.4V
    1080,                       // volt th 2 = 10.8V
    1020,                       // volt th 1 = 10.2V
    960,                        // volt th 1b = 9.6V
    20,                         // volt hysteresis
    FSR_VOLTAGE_FACTOR,         // voltage factor
    FSR_CURRENT_FACTOR          // current factor
};



FSRDevice::FSRDevice(DYNAMIXEL::SerialPortHandler &dxl_port)
  :DYNAMIXEL::Slave(dxl_port, DEF_MODEL_NUMBER, 2.0)
{
}

/**
 * @brief Activates the pins used by the device.
 * 
 */
void FSRDevice::setupPins(void)
{
    // Configure the ADC pin
    pinMode(FSR_FR, INPUT_ANALOG);
    pinMode(FSR_FL, INPUT_ANALOG);
    pinMode(FSR_BR, INPUT_ANALOG);
    pinMode(FSR_BL, INPUT_ANALOG);
    pinMode(CURR,   INPUT_ANALOG);
    pinMode(VOLT,   INPUT_ANALOG);
    analogReadResolution(12);       // we set this explicitly as by default is 10b

    pinMode(LED1,   OUTPUT); // LED 1 MSB
    pinMode(LED2,   OUTPUT); // LED 2
    pinMode(LED3,   OUTPUT); // LED 3
    pinMode(LED4,   OUTPUT); // LED 4 LSB
    pinMode(BUZZER, OUTPUT); // buzzer
    pinMode(CUTOFF, OUTPUT); // power enable
}


/**
 * @brief Attempts to read the whole ROM from Flash and if the model_number
 * and firmware_version are not the ones used by this firmware it will
 * reset the whole ROM to the defaults. Then it will read the ROM content
 * and store it in the `rom` attribute of the device to be used by various
 * methods.
 */
void FSRDevice::readConfigFromROM(void)
{
    // uint16_t    model_number;
    // uint8_t     firmware_version;

#ifdef FORCE_RELOAD_ROM
    EEPROM.put(0, defaultROM);
#endif
    EEPROM.get(0, rom);
    
    if ( rom.model_number != DEF_MODEL_NUMBER || rom.firmware_version != DEF_FIRMWARE_VERSION) {
        EEPROM.put(0, defaultROM);
        EEPROM.get(0, rom);
    }

// #else
//     // first try to read the model and firmware to see if we need to
//     // initialize the ROM to factory settings
//     EEPROM.get(0, model_number);
//     EEPROM.get(6, firmware_version);
//     if ( model_number != DEF_MODEL_NUMBER || firmware_version != DEF_FIRMWARE_VERSION) {
//         EEPROM.put(0, defaultROM);
//     }
//     // read the whole ROM
//     EEPROM.get(0, rom);
// #endif
    // configure the device - call the Slave methods to set the variables
    setFirmwareVersion(rom.firmware_version);
    setID(rom.id);
    setPortProtocolVersionUsingIndex(rom.protocol_type);

}


/**
 * @brief Registers the control items for the Slave framework to be able to
 * process them in write and read functions. `model_number`, `firmware_version`,
 * `id` and `protocol_version` are added to the control items by the Slave
 * constructor automatically. We need to add the rest of the items.
 */
void FSRDevice::addControlItems(void)
{
    // ADDR_ITEM_MODEL_NUMBER added by Slave
    // ADDR_ITEM_FIRMWARE_VERSION added by Slave
    // ADDR_ITEM_ID added by Slave
    // ADDR_ITEM_PROTOCOL_VER added by Salve
    addControlItem(ADDR_ITEM_MODEL_INFO, rom.model_information);
    addControlItem(ADDR_ITEM_BAUD_RATE, rom.baud_rate);
    addControlItem(ADDR_ITEM_RETURN_DELAY_TIME, rom.return_delay_time);
    addControlItem(ADDR_ITEM_VOLTAGE_SAMPLES, rom.voltage_samples);
    addControlItem(ADDR_ITEM_VOLTAGE_TH4, rom.voltage_th4);
    addControlItem(ADDR_ITEM_VOLTAGE_TH3, rom.voltage_th3);
    addControlItem(ADDR_ITEM_VOLTAGE_TH2, rom.voltage_th2);
    addControlItem(ADDR_ITEM_VOLTAGE_TH1, rom.voltage_th1);
    addControlItem(ADDR_ITEM_VOLTAGE_TH1B, rom.voltage_th1b);
    addControlItem(ADDR_ITEM_VOLTAGE_HYST, rom.voltage_hyst);
    addControlItem(ADDR_ITEM_VOLTAGE_FACTOR, rom.voltage_factor);
    addControlItem(ADDR_ITEM_CURRENT_FACTOR, rom.current_factor);

    // RAM
    addControlItem(ADDR_ITEM_VOLTAGE, ram.voltage);
    addControlItem(ADDR_ITEM_CURRENT, ram.current);
    addControlItem(ADDR_ITEM_BATTERY_PERC, ram.battery_perc);
    addControlItem(ADDR_ITEM_LOOP_RATE, ram.loop_rate);
}


uint32_t FSRDevice::getBaudrateValueFromIndex(uint8_t baud_index)
{
  switch(baud_index)
  {
    case 0: return 9600;
    case 1: return 57600;
    case 2: return 115200;
    case 3: return 1000000;
    case 4: return 2000000;
    case 5: return 3000000;
    case 6: return 4000000;
    case 7: return 4500000;
    default: return 0;
  }
}


uint8_t FSRDevice::getErrorDataRange()
{
  if (getPortProtocolVersionIndex() == 2)
    return DXL2_0_ERR_DATA_RANGE;
  else
    return 1 << DXL1_0_ERR_RANGE_BIT;
}


/**
 * @brief Starts the communication port with the speed provided from ROM.
 * 
 * @param dxl_port the SerialPortHandler that manages the communication for this
 * device.
 */
void FSRDevice::startDXL()
{
    uint32_t br = getBaudrateValueFromIndex(rom.baud_rate);
    ((DYNAMIXEL::SerialPortHandler *)getPort())->begin(br);
    // dxl_port.begin(br);
}


void FSRDevice::write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
    (void)dxl_err_code, (void)arg;

    switch(item_addr)
    {
        case ADDR_ITEM_ID:
            // value is already checked by the Salve class for correct range
            rom.id = getID();
            EEPROM.put(0, rom);
            break;

        case ADDR_ITEM_PROTOCOL_VER:
            // value is alaready checked by the Slave class for correct values
            rom.protocol_type = getPortProtocolVersionIndex();
            EEPROM.put(0, rom);
            break;

        case ADDR_ITEM_BAUD_RATE:
            if (getBaudrateValueFromIndex(rom.baud_rate) == 0) {
                dxl_err_code = getErrorDataRange();
            }
            else if (rom.baud_rate > 4) {
                // for the time being we do not support speeds > 2mbps because 
                // of the serial communication problems
                dxl_err_code = getErrorDataRange();
            }
            else {
                EEPROM.put(0, rom);
                ((DYNAMIXEL::SerialPortHandler *)getPort())->begin(getBaudrateValueFromIndex(rom.baud_rate));
            }
            break;

        case ADDR_ITEM_RETURN_DELAY_TIME:
            if(rom.return_delay_time > 250)
                dxl_err_code = getErrorDataRange();
            else
                EEPROM.put(0, rom);
            break;
    }
}


void FSRDevice::read_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;

  switch(item_addr) {
    // case ADDR_ITEM_CAL_FL: {
    //   uint32_t mul = lpf_raw_fl * fl_mul;
    //   lpf_cal_fl = (uint16_t)(mul / fl_div);
    //   break;
    // };
//
//
//    case ADDR_CONTROL_FSR_1: {
//      control_fsr_1 = analogRead(PA1);
//      break;
//    };
//
//    case ADDR_CONTROL_FSR_2: {
//      control_fsr_2 = analogRead(PA2);
//      break;
//    };
//
//    default:
//      if (dxl.getPortProtocolVersion() == 2.0){
//        dxl_err_code = DXL1_0_ERR_INSTRUCTION_BIT;
//        break;
//      }
//      else {
//        dxl_err_code = DXL2_0_ERR_ACCESS;
//        break;
//      }
  }
  
}


/**
 * @brief Reads voltage and current from the ADC ports, converts them to 
 * the reporting units and saves them in the RAM using a low pass filter
 * simple averaging over a number of events that can be specified in the 
 * `voltage_samples` in ROM.
 */
bool FSRDevice::updateVoltage(void)
{
    static uint16_t samples = 0;            // number of samples read
    static uint32_t raw_voltage_sum = 0;    // sum for the voltage readings
    static uint32_t raw_current_sum = 0;    // sum for the current readings
    // uint16_t        conv;                   // converted value

    raw_voltage_sum += analogRead(VOLT);
    raw_current_sum += analogRead(CURR);
    samples++;

    if (samples == rom.voltage_samples) {
        // ram.voltage_raw = raw_voltage_sum / samples;
        ram.voltage = raw_voltage_sum / samples * rom.voltage_factor / FSR_ADC_RESOLUTION;
        // ram.current_raw = raw_current_sum / samples;
        ram.current = raw_current_sum / samples * rom.current_factor / FSR_ADC_RESOLUTION;
        samples = 0;
        raw_voltage_sum = 0;
        raw_current_sum = 0;
    }
    return (samples == 0);      // this means we have updated values
}


/**
 * @brief A state machine that implements the transition between the possible
 * states of the battery, taking into account hysteresis to avoid flickering
 * of state reporting. The stages are controlled by thresholds that can be
 * configured in the ROM (`voltage_thXX`) as well as the amount of hysteresis
 * (`voltage_hyst`). Moving downwards in the state space is done simply when
 * the threshold are passed. Moving up the state space require the threshold
 * plus the hysteresis to be passed.
 */
void FSRDevice::updateBatteryState(void)
{
    switch(battery_state_) {
        case STATE4:    // state: 4 leds
            if (ram.voltage < rom.voltage_th4) {
                battery_state_ = STATE3;
            }
            break;

        case STATE3:    // state: 3 leds
            if (ram.voltage < rom.voltage_th3) {
                battery_state_ = STATE2;
            }
            if (ram.voltage >= (rom.voltage_th4 + rom.voltage_hyst)) {
                battery_state_ = STATE4;
            }
            break;

        case STATE2:    // state 2 leds
            if (ram.voltage < rom.voltage_th2) {
                battery_state_ = STATE1;
            }
            if (ram.voltage >= (rom.voltage_th3 + rom.voltage_hyst)) {
                battery_state_ = STATE3;
            }
            break;

        case STATE1:    // state 1 led
            if (ram.voltage < rom.voltage_th1) {
                battery_state_ = STATE1B;
            }
            if (ram.voltage >= (rom.voltage_th2 + rom.voltage_hyst)) {
                battery_state_ = STATE2;
            }
            break;

        case STATE1B:   // state 1 led blinking
            if (ram.voltage < rom.voltage_th1b) {
                battery_state_ = STATESD;
            }
            if (ram.voltage >= (rom.voltage_th1 + rom.voltage_hyst)) {
                battery_state_ = STATE1;
            }
            break;

        case STATESD:   // state 1 led flashing (shut-down)
            // to move from shutdown the hysteresis is multiplied by 3
            // this is because the changes in the battery voltage when
            // the consumer is taken down can be substantial and can put the
            // the supply in oscillating mode
            if (ram.voltage >= (rom.voltage_th1b + 3 * rom.voltage_hyst)) {
                battery_state_ = STATE1B;
            }
            break;
    }
}


/**
 * @brief Shows the state of the battery using the 4 LEDs and the buzzer.
 * For above TH4 all 4 LEDs are on.
 * For between TH3 and TH4 3 LEDs are on.
 * For between TH2 and TH3 2 LEDs are on.
 * For between TH1 and TH2 1 LED is on.
 * For TH1B and TH LED1 will blink with 50% fill rate at 1Hz and buzzer will
 * sound with the same schedule.
 * Bellow TH1B the LED1 will blink with 10% fill rate at 1Hz, no buzzer will
 * sound (to conserve energy) and the Cut-off circuit for the battery will
 * be triggered to protect the battery from over-discharging.
 */
void FSRDevice::displayBatteryState(void)
{
    static uint16_t last_state = 1;
    static bool     blink = false;

    if (battery_state_ != last_state) {
        if (battery_state_ == STATE4) {
            LEDON(4); LEDON(3); LEDON(2); LEDON(1);
            noTone(BUZZER); noCutoff;
        }
        if (battery_state_ == STATE3) {
            LEDOFF(4); LEDON(3); LEDON(2); LEDON(1);
            noTone(BUZZER); noCutoff;
        }
        if (battery_state_ == STATE2) {
            LEDOFF(4); LEDOFF(3); LEDON(2); LEDON(1);
            noTone(BUZZER); noCutoff;
        }
        if (battery_state_ == STATE1) {
            LEDOFF(4); LEDOFF(3); LEDOFF(2); LEDON(1);
            noTone(BUZZER); noCutoff;
        }
        if (battery_state_ == STATE1B) {
            LEDOFF(4); LEDOFF(3); LEDOFF(2);
            noCutoff;
        }
        if (battery_state_ == STATESD) {
            LEDOFF(4); LEDOFF(3); LEDOFF(2);
            noTone(BUZZER); cutoff;
        }
        last_state = battery_state_;
    }
    // blink for 1B
    if (battery_state_ == STATE1B) {
        if ((millis() % 1000) < 500 && !blink) {
            LEDON(1);
            tone(BUZZER, 500);
            blink = true;
        }
        if ((millis() % 1000) >= 500 && blink) {
            LEDOFF(1);
            noTone(BUZZER);
            blink = false;
        }
    }
    // blink for SD
    if (battery_state_ == STATESD) {
        if ((millis() % 1000) < 100 && !blink) {
            LEDON(1);
            blink = true;
        }
        if ((millis() % 1000) >= 100 && blink) {
            LEDOFF(1);
            blink = false;
        }
    }
}


void FSRDevice::processFSR(void)
{
    static uint16_t samples = 0;            // number of samples read
    static uint32_t raw_fl_sum = 0;         // sum for the FL readings
    static uint32_t raw_fr_sum = 0;         // sum for the FL readings
    static uint32_t raw_bl_sum = 0;         // sum for the BL readings
    static uint32_t raw_br_sum = 0;         // sum for the BR readings
    uint16_t        conv;                   // converted value

    raw_fl_sum += analogRead(FSR_FL);
    raw_fr_sum += analogRead(FSR_FR);
    raw_bl_sum += analogRead(FSR_BL);
    raw_br_sum += analogRead(FSR_BR);
    samples++;
}


void FSRDevice::processStatistics(void)
{
    static uint32_t last_update = millis();
    static uint16_t no_loops = 0;

    no_loops++;
    uint32_t current = millis();
    if ((current - last_update) >= 1000) {
        ram.loop_rate = no_loops;
        no_loops = 0;
        last_update = current;
    }

}