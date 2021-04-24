/*
 * app_common.h
 *
 *  Created on: Apr 8, 2021
 *      Author: hanguyen
 */

#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define LED_GREEN_ON()				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
#define LED_GREEN_OFF()				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
#define LED_GREEN_BLINK()			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

#define LED_ORANGE_ON()				HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
#define LED_ORANGE_OFF()			HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
#define LED_ORANGE_BLINK()			HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);

#define LED_RED_ON()				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
#define LED_RED_OFF()				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
#define LED_RED_BLINK()				HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

#define LED_BLUE_ON()				HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
#define LED_BLUE_OFF()				HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
#define LED_BLUE_BLINK()			HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

#endif /* APP_COMMON_H */
