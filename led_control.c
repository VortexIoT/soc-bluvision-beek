/***************************************************************************//**
 * @file
 * @brief Led control for managing led hardware
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

#include "led_control.h"

#include "em_gpio.h"
#include "em_timer.h"
#include "em_cmu.h"

#include "hal-config.h"



/***********************************************************************************************//**
 * @addtogroup LED
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup led_control
 * @{
 **************************************************************************************************/

/* LED polarity is active-low in those radio boards that share same pin for BUTTON/LED */
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
#define LED_ACTIVE_LOW
#define LED_OFF_STATE 1
#define LED_ON_STATE  0
#else
#define LED_OFF_STATE 0
#define LED_ON_STATE  1
#endif


/*******************************************************************************
 * Initialize LEDs hardware.
 ******************************************************************************/
void LEDS_control_init(void)
{
  // configure LED pins
  GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, LED_OFF_STATE);
  GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, LED_OFF_STATE);
}

/*******************************************************************************
 * Set LED color based on lightness and temperature.
 *
 * @param[in] level        Lightness level.
 * @param[in] temperature  Color temperature in Kelvins.
 *
 * @note On single color LEDs only lightness is changed.
 ******************************************************************************/
void LEDS_SetColor(uint16_t level, uint16_t temperature)
{
  /* all LED adjustments go trough this function. The polarity is taken into account by
   * flipping the level setting for boards with active-low LED control */
#ifdef LED_ACTIVE_LOW
  level = 0xFFFF - level;
#endif

  /* for simplicity, LEDs are driven with PWM in all states, even 0% and 100%.
   * Therefore need to limit the min/max value used for the timer CC register */
  if (level < MIN_BRIGHTNESS) {
    level = MIN_BRIGHTNESS;
  } else if (level > MAX_BRIGHTNESS) {
    level = MAX_BRIGHTNESS;
  }

  TIMER_CompareSet(TIMER0, 0, level);
  TIMER_CompareSet(TIMER0, 1, level);
}

/** @} (end addtogroup led_control) */
/** @} (end addtogroup LED) */
