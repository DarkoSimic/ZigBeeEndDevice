#ifndef __SPI_H__
#define __SPI_H__

#define PART_LM3S9B90

#include "inc/lm3s9b90.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"

#include "utils/uartstdio.h"
#include "driverlib/uart.h"

#include <stdint.h>

#define SPI_READ													0x80
#define SPI_WRITE													0x7F
#define SPI_BUFFER_LEN 											26
#define SPI_BUFFER_LEN_WRITE								 4

#define BME280_DATA_INDEX										 1
#define BME280_ADDRESS_INDEX 							   2

#define CS									 GPIO_PORTD_DATA_R

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

extern void initSPI(void);
extern void initConsole(void);
extern void bme280SendSPI(unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt);
extern void bme280ReadSPI(unsigned char reg_address, unsigned char *reg_data, unsigned char cnt);
extern void cleanFIFO(void);
extern tBoolean statusSPI(unsigned long ulBase);

#endif // __SPI_H__
