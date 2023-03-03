const uint16_t  DEF_MODEL_NUMBER        = 0x5301;
const uint32_t  DEF_MODEL_INFO          = 20220118;
const uint8_t   DEF_FIRMWARE_VERSION    = 0x06;


// registers ROM
struct ROM {
    uint16_t    model_number;       // 0[2]
    uint32_t    model_information;  // 2[4]
    uint8_t     firmware_version;   // 6[1]
    uint8_t     id;                 // 7[1]
    uint8_t     baud_rate;          // 8[1]
    uint8_t     protocol_type;      // 9[1]
    uint8_t     return_delay_time;  // 13[1]
    uint16_t    voltage_samples;    // 14[2]
    uint16_t    voltage_th4;        // 16[2]
    uint16_t    voltage_th3;        // 18[2]
    uint16_t    voltage_th2;        // 20[2]
    uint16_t    voltage_th1;        // 22[2]
    uint16_t    voltage_th1b;       // 24[2]
    uint16_t    voltage_hyst;       // 26[2]
    uint16_t    voltage_factor;     // 28[2]
    uint16_t    current_factor;     // 30[2]
};


// registers RAM
struct RAM {
    uint16_t    voltage;            // 70
    uint16_t    current;            // 72
    uint8_t     battery_perc;       // 74

    uint16_t    loop_rate;          // 98
};
