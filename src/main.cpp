#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "fsr_device.h"


// extern "C" void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
//   RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

//   /* Initializes the CPU, AHB and APB busses clocks */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//     Error_Handler();
//   }

//   /* Initializes the CPU, AHB and APB busses clocks */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
//                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
//     Error_Handler();
//   }

//   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
//   PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
//   PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
//   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
//     Error_Handler();
//   }
// }


DYNAMIXEL::SerialPortHandler    dxl_port(DXL_PORT, DXL_DIR_PIN);
FSRDevice                       device(dxl_port);


void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
    device.write_callback_func(item_addr, dxl_err_code, arg);
}


void read_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
    device.read_callback_func(item_addr, dxl_err_code, arg);
}


void setup() {
    device.setupPins();
    device.readConfigFromROM();
    device.addControlItems();
    device.startDXL();
    // register callbacks
    device.setWriteCallbackFunc(write_callback_func);
    device.setReadCallbackFunc(read_callback_func);
}


void loop() {
    // read voltages / current
    if (device.updateVoltage())
        device.updateBatteryState();
    device.displayBatteryState();

    device.processFSR();

    device.processStatistics();

    //DXL process
    device.processPacket();
}