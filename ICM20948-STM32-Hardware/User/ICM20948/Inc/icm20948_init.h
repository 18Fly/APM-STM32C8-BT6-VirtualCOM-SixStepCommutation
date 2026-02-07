#ifndef __ICM20948_INIT_H
#define __ICM20948_INIT_H

#include "gpio.h"
#include "icm20948_def.h"
#include "spi.h"
#include <stdint.h>

#define ICM20948_CS_PORT GPIOA
#define ICM20948_CS_PIN GPIO_PIN_4
#define ICM20948_TRUE_ID 0xEA

void ICM20948_Select();

void ICM20948_Deselect();

int icm20948_initialize(void);

void icm20948_fifo_enable(void);

void icm20948_read_fifo(uint8_t *buffer);

void icm20948_read_raw_data(uint8_t *buffer);

uint8_t SPI1_ReadWriteByte(uint8_t txData);

uint8_t ICM20948_WAI(void);

uint8_t ICM20948_IsDataReady(void);

#endif // !__ICM20948_INIT_H
