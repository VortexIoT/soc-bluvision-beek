/***************************************************************************//**
 * @file
 * @brief Silicon Labs Thermometer Example Application
 * This Thermometer and OTA example allows the user to measure temperature
 * using the temperature sensor on the WSTK. The values can be read with the
 * Health Thermometer reader on the Blue Gecko smartphone app.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>

/* Board Headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* LED driver with support for PWM dimming */
#include "led_driver.h"
#include "led_control.h"

/* Device initialization header */
#include "hal-config.h"

#ifdef FEATURE_BOARD_DETECTED
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#else
#error This sample app only works with a Silicon Labs Board
#endif

#include "i2cspm.h"
#include "si7013.h"
#include "tempsens.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* Gecko configuration parameters (see gecko_configuration.h) */
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

static const gecko_configuration_t config = {
  .config_flags = 0,
#if defined(FEATURE_LFXO)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#else
  .sleep.flags = 0,
#endif // LFXO
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,                 /* Enable antenna configuration. */
  .rf.antenna = GECKO_RF_ANTENNA,                      /* Select antenna path! */
};

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

uint8_t LEDS_State = 0; //0 indicates LEDs are off. 1 indicates LEDs are on.

//The address here is to be supplied in the reverse order. It is used in little endian order.
//bd_addr BluVisionDeviceAddress = {0xCC, 0x79, 0xDB, 0x57, 0x8D, 0x56};
bd_addr BluVisionDeviceAddress = {0x56, 0x8D, 0x57, 0xDB, 0x79, 0xCC};
//bd_addr BluVisionDeviceAddress = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

uint8 BluVisionAdHandle = 0;
uint8 BluVisionAdScanResponse = 0;
uint8 BluVisionAdLength = 31;

uint32_t BluVisionDeviceTime = 27258000;

//This advertisement data contains the manufacturer data for time instant when this ad was captured on the android app
//uint8 BluVisionAdData[] = {0x02,0x01,0x06,0x03,0x02,0x04,0x18,0x17,0xFF,0xF9,0x00,0x01,0xCF,0xB5,0xD3,0x69,0x9C,0xD0,0x60,0x01,0xC3,0xCB,0x8F,0x01,0xA1,0xF1,0x17,0x0F,0xA1,0x8F,0xDB};

//This advertisement data contains an incremented device time stamp in the manufacturer specific data
uint8 BluVisionAdData[] = {0x02,0x01,0x06,0x03,0x02,0x04,0x18,0x17,0xFF,0xF9,0x00,0x01,0xCF,0xB5,0xD3,0x69,0x9C,0xD0,0x60,0x01,0x50,0xE6,0x9F,0x01,0xA1,0xF1,0x17,0x0F,0xA1,0x8F,0xDB};

//This is the scan response data which forms the additional 31 bytes
uint8 BluVisionScanResponseData[] = {0x06,0x08,0x74,0x69,0x73,0x35,0x32,0x11,0x06,0xA6,0xDA,0x37,0xDE,0xC1,0x9A,0xFC,0x80,0x94,0x4A,0xD8,0xA8,0x02,0x62,0xC2,0xBE,0x02,0x0A,0x00,0x00,0x00,0x00};


/**
 * @brief Function for taking a single temperature measurement with the WSTK Relative Humidity and Temperature (RHT) sensor.
 */
void temperatureMeasure()
{
  uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
  uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
  int32_t tempData;     /* Stores the Temperature data read from the RHT sensor. */
  uint32_t rhData = 0;    /* Dummy needed for storing Relative Humidity data. */
  uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
  uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
  static int32_t DummyValue = 0l; /* This dummy value can substitute the temperature sensor value if the sensor is N/A. */

  /* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
  UINT8_TO_BITSTREAM(p, flags);

  /* Sensor relative humidity and temperature measurement returns 0 on success, nonzero otherwise */
  if (Si7013_MeasureRHAndTemp(I2C0, SI7021_ADDR, &rhData, &tempData) != 0) {
    /* Use the dummy value and go between 20 and 40 if the sensor read failed.
     * The ramp-up value will be seen in the characteristic in the receiving end. */
    tempData = DummyValue + 20000l;
    DummyValue = (DummyValue + 1000l) % 21000l;
  }
  /* Convert sensor data to correct temperature format */
  temperature = FLT_TO_UINT32(tempData, -3);
  /* Convert temperature to bitstream and place it in the HTM temperature data buffer (htmTempBuffer) */
  UINT32_TO_BITSTREAM(p, temperature);

  /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
   * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
   *  0xFF as connection ID will send indications to all connections. */
  //gecko_cmd_gatt_server_send_characteristic_notification(
  //  0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
}

/**
 * @brief  Main function
 */
int main(void)
{
  // Initialize device
  initMcu();

  // Initialize board including LED definitions
  initBoard();

  // Initialize application
  initApp();

  initVcomEnable();

  // Initialize stack
  gecko_init(&config);


  // Initialize the Temperature Sensor
  Si7013_Detect(I2C0, SI7021_ADDR, NULL);


  gecko_cmd_system_set_bt_address(BluVisionDeviceAddress);

  //Call a timer for 1000ms, Event Handle 0 and in Repeating Mode
  //parameters are (uint32 time, uint8 handle, uint8 single_shot)
  //time is measured in hardware clock ticks, where 32768 ticks is 1 second
  //3277 is closest to 100 ms or 10Hz
  gecko_cmd_hardware_set_soft_timer(32768, 0, 0);


  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
        /* Set advertising parameters. 100ms advertisement interval.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). */
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

        /* Set advertisement data as per BEEK advertisements */
        //gecko_cmd_le_gap_bt5_set_adv_data(BluVisionAdHandle, BluVisionAdScanResponse, BluVisionAdLength, BluVisionAdData);

        /* Start general advertising and enable connections. */

        gecko_cmd_le_gap_bt5_set_adv_data(0, 0, 31, BluVisionAdData);
        gecko_cmd_le_gap_bt5_set_adv_data(0, 1, 31, BluVisionScanResponseData);
        gecko_cmd_le_gap_start_advertising(0, le_gap_user_data, le_gap_connectable_scannable);
        //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      /* This event is generated when a connected client has either
       * 1) changed a Characteristic Client Configuration, meaning that they have enabled
       * or disabled Notifications or Indications, or
       * 2) sent a confirmation upon a successful reception of the indication. */
      case gecko_evt_gatt_server_characteristic_status_id:
        /* Check that the characteristic in question is temperature - its ID is defined
         * in gatt.xml as "temperature_measurement". Also check that status_flags = 1, meaning that
         * the characteristic client configuration was changed (notifications or indications
         * enabled or disabled). */
        //if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
        //    && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
        //  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02) {
        //    /* Indications have been turned ON - start the repeating timer. The 1st parameter '32768'
        //     * tells the timer to run for 1 second (32.768 kHz oscillator), the 2nd parameter is
        //     * the timer handle and the 3rd parameter '0' tells the timer to repeat continuously until
        //     * stopped manually.*/
        //    gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
        //  } else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00) {
        //    /* Indications have been turned OFF - stop the timer. */
        //    gecko_cmd_hardware_set_soft_timer(0, 0, 0);
        //  }
        //}
        break;

      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
      case gecko_evt_hardware_soft_timer_id:
        /* Measure the temperature as defined in the function temperatureMeasure() */
        //temperatureMeasure();

    	/* Set or reset the LED */
    	if(LEDS_State == 1) {
    		//LEDS_SetState(LED_STATE_OFF);
    		LEDS_State = 0;
    		GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, 0);
    		GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, 1);
    		printf("LED\n");

    	} else {
    		//LEDS_SetState(LED_STATE_ON);
    		LEDS_State = 1;
    		GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, 1);
    		GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, 0);
    	}
        break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Stop timer in case client disconnected before indications were turned off */
          //gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          /* Restart advertising after client has disconnected */
          //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
          gecko_cmd_le_gap_start_advertising(0, le_gap_user_data, le_gap_connectable_scannable);
        }
        break;


      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Checks if the user-type OTA Control Characteristic was written.
       * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
      // case gecko_evt_gatt_server_user_write_request_id:
      //  if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
      //    /* Set flag to enter to OTA mode */
      //    boot_to_dfu = 1;
      //    /* Send response to Write Request */
      //    gecko_cmd_gatt_server_send_user_write_response(
      //      evt->data.evt_gatt_server_user_write_request.connection,
      //      gattdb_ota_control,
      //      bg_err_success);

          /* Close connection to enter to DFU OTA mode */
      //    gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      //  }
      //  break;*/

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
