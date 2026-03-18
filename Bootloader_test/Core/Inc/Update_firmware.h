/*
 * Update_firmware.h
 *
 *  Created on: Mar 3, 2026
 *      Author: Nguyen Hoang Quan
 */

#ifndef INC_UPDATE_FIRMWARE_H_
#define INC_UPDATE_FIRMWARE_H_
#include "stm32f4xx_hal.h"
#include "application.h"

#define APP_ADDR 0x08008000U

HAL_StatusTypeDef erase_app(uint32_t fw_size);
HAL_StatusTypeDef write_app(uint32_t address, const uint8_t *data, uint32_t length);
int verify_app(uint32_t address, const uint8_t *data, uint32_t length);
void bootloader_update();

#endif /* INC_UPDATE_FIRMWARE_H_ */
