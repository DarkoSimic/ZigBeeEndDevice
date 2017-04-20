#include "spi.h"


void initConsole(void)
{
	
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | 
					SYSCTL_XTAL_16MHZ);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | 
							UART_CONFIG_PAR_NONE));
	
	UARTEnable(UART0_BASE);
}
	

//*****************************************************************************
//
//! Initialization and configuration of the SPI interface
//! as a master, SSI0 mode at  port A, with data rate 1 Mbps 
//! and data size of 8 bits
//
//*****************************************************************************
void initSPI(void)
{

//*****************************************************************************
//
//! Enables a peripheral.
//!
//! \param ulPeripheral is the peripheral to enable.
//!
//! This function enables peripherals.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//!
//! \b SYSCTL_PERIPH_GPIOA, \b SYSCTL_PERIPH_GPIOE, \b SYSCTL_PERIPH_GPIOF,
//! \b SYSCTL_PERIPH_GPIOH, \b SYSCTL_PERIPH_SSI0,
//! \b SYSCTL_PERIPH_SSI1
//!
//! \note It takes five clock cycles after the write to enable a peripheral
//! before the the peripheral is actually enabled.  During this time, attempts
//! to access the peripheral result in a bus fault.  Care should be taken
//! to ensure that the peripheral is not accessed during this brief time
//! period.
//!
//! \return None.
//
//*****************************************************************************
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
//*****************************************************************************
//
//! Configures the alternate function of a GPIO pin.
//!
//! \param ulPinConfig is the pin configuration value, specified as only one of
//! the \b GPIO_P??_??? values.
//!
//! This function configures the pin mux that selects the peripheral function
//! associated with a particular GPIO pin.  Only one peripheral function at a
//! time can be associated with a GPIO pin, and each peripheral function should
//! only be associated with a single GPIO pin at a time (despite the fact that
//! many of them can be associated with more than one GPIO pin). To fully
//! configure a pin, a GPIOPinType*() function should also be called.
//!
//! The available mappings are supplied on a per-device basis in
//! <tt>pin_map.h</tt>.  The \b PART_IS_<partno> define enables the
//! appropriate set of defines for the device that is being used.
//!
//! \note This function is not valid on Sandstorm, Fury, and Dustdevil-class
//! devices. Also, if the same signal is assigned to two different GPIO port
//! pins, the signal is assigned to the port with the lowest letter and the
//! assignment to the higher letter port is ignored.
//!
//! \return None.
//
//*****************************************************************************	
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

//*****************************************************************************
//
//! Configures pin(s) for use by the SSI peripheral.
//!
//! \param ulPort is the base address of the GPIO port.
//! \param ucPins is the bit-packed representation of the pin(s).
//!
//! The SSI pins must be properly configured for the SSI peripheral to function
//! correctly.  This function provides a typical configuration for those
//! pin(s); other configurations may work as well depending upon the board
//! setup (for example, using the on-chip pull-ups).
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that is
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//!
//! \note This function cannot be used to turn any pin into a SSI pin; it only
//! configures a SSI pin for proper operation. Devices with flexible pin
//! muxing also require a GPIOPinConfigure() function call.
//!
//! \return None.
//
//*****************************************************************************
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
					GPIO_PIN_2);

//*****************************************************************************
//
//! Disables the synchronous serial interface.
//!
//! \param ulBase specifies the SSI module base address.
//!
//! This function disables operation of the synchronous serial interface.
//!
//! \return None.
//
//*****************************************************************************
	SSIDisable(SSI0_BASE);
					
//*****************************************************************************
//
//! Configures the synchronous serial interface.
//!
//! \param ulBase specifies the SSI module base address.
//! \param ulSSIClk is the rate of the clock supplied to the SSI module.
//! \param ulProtocol specifies the data transfer protocol.
//! \param ulMode specifies the mode of operation.
//! \param ulBitRate specifies the clock rate.
//! \param ulDataWidth specifies number of bits transferred per frame.
//!
//! This function configures the synchronous serial interface.  It sets
//! the SSI protocol, mode of operation, bit rate, and data width.
//!
//! The \e ulProtocol parameter defines the data frame format.  The
//! \e ulProtocol parameter can be one of the following values:
//! \b SSI_FRF_MOTO_MODE_0, \b SSI_FRF_MOTO_MODE_1, \b SSI_FRF_MOTO_MODE_2,
//! \b SSI_FRF_MOTO_MODE_3, \b SSI_FRF_TI, or \b SSI_FRF_NMW.  The Motorola
//! frame formats encode the following polarity and phase configurations:
//!
//! <pre>
//! Polarity Phase       Mode
//!   0       0   SSI_FRF_MOTO_MODE_0
//!   0       1   SSI_FRF_MOTO_MODE_1
//!   1       0   SSI_FRF_MOTO_MODE_2
//!   1       1   SSI_FRF_MOTO_MODE_3
//! </pre>
//!
//! The \e ulMode parameter defines the operating mode of the SSI module.  The
//! SSI module can operate as a master or slave; if it is a slave, the SSI can
//! be configured to disable output on its serial output line.  The \e ulMode
//! parameter can be one of the following values: \b SSI_MODE_MASTER,
//! \b SSI_MODE_SLAVE, or \b SSI_MODE_SLAVE_OD.
//!
//! The \e ulBitRate parameter defines the bit rate for the SSI.  This bit rate
//! must satisfy the following clock ratio criteria:
//!
//! - FSSI >= 2 * bit rate (master mode); this speed cannot exceed 25 MHz.
//! - FSSI >= 12 * bit rate or 6 * bit rate (slave modes), depending on the
//! capability of the specific microcontroller
//!
//! where FSSI is the frequency of the clock supplied to the SSI module.
//!
//! The \e ulDataWidth parameter defines the width of the data transfers and
//! can be a value between 4 and 16, inclusive.
//!
//! The peripheral clock is the same as the processor clock.  This value is
//! returned by SysCtlClockGet(), or it can be explicitly hard coded if it is
//! constant and known (to save the code/execution overhead of a call to
//! SysCtlClockGet()).
//!
//! This function replaces the original SSIConfig() API and performs the same
//! actions.  A macro is provided in <tt>ssi.h</tt> to map the original API to
//! this API.
//!
//! \return None.
//
//*****************************************************************************
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
						SSI_MODE_MASTER, 100000, 8);

//*****************************************************************************
//
//! Enables the synchronous serial interface.
//!
//! \param ulBase specifies the SSI module base address.
//!
//! This function enables operation of the synchronous serial interface.  The
//! synchronous serial interface must be configured before it is enabled.
//!
//! \return None.
//
//*****************************************************************************
	SSIEnable(SSI0_BASE);
	
//*****************************************************************************
//
//! Enable external CSB on portD, pin1
//
//*****************************************************************************
	
	//
	// Enable portD
	// Enable portE
	//
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;
	
	//
	// Delay 1.5 [us]
	//
	SysCtlDelay(8);
	
	//
	// Set Pin1 as output - chip select (SS) for SPI
	//
	HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) 	= 0x00000002;
	SysCtlDelay(1);
 	
	//
	// Pin1 digital enable
	//
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN)		= 0x00000002;
	SysCtlDelay(1);
	GPIO_PORTD_DATA_R = 0x02;
	
	//
	// Set Pin1 as output - control pin, ON heating
	// Set Pin2 as output - control pin, ON aircond.
	//
	HWREG(GPIO_PORTE_BASE + GPIO_O_DIR) = 0x06;
	SysCtlDelay(1);
	
	//
	// Pin1 and Pin2 digital enable
	//  
	HWREG(GPIO_PORTE_BASE + GPIO_O_DEN) = 0x06;
	SysCtlDelay(1);
	GPIO_PORTE_DATA_R = 0x00;
}

void bme280ReadSPI(unsigned char reg_address, unsigned char *reg_data, unsigned char cnt)
{
	int init = 0;
	unsigned long ulTmp;
	unsigned char array [SPI_BUFFER_LEN] = {0};
	unsigned char stringpos; 
	
	cleanFIFO();
	
	array[init] = reg_address | SPI_READ;
	
	/*
	 * CS = '0'
	 */
	CS = 0x00;
	
	SSIDataPut(SSI0_BASE, array[init]);
	while(SSIBusy(SSI0_BASE))
	{
	}
	SSIDataGet(SSI0_BASE, &ulTmp);
	
	for (stringpos = init; stringpos < cnt; stringpos++) 
	{
		SSIDataPut(SSI0_BASE, 0xAA);
		while(SSIBusy(SSI0_BASE))
		{
		} 
		
		SSIDataGet(SSI0_BASE, &ulTmp);
		array[stringpos + BME280_DATA_INDEX] = ulTmp & 0xff;
		*(reg_data + stringpos) = array[stringpos+BME280_DATA_INDEX];
	}
	
	/*
	 * CS = '1'
	 */
	CS = 0x02;
	
	
}// void readSPI(unsigned char reg_address, unsigned char *reg_data, unsigned char cnt);

void bme280SendSPI(unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	unsigned char init = 0;
	unsigned char array[SPI_BUFFER_LEN_WRITE * BME280_ADDRESS_INDEX];
	unsigned char	stringpos = 0;
	unsigned char index = 0;
	
	/*
	 * CS = '0'
	 */
	CS = 0x00;
	
	for (stringpos = init; stringpos < cnt; stringpos++) 
	{
		/* the operation of (reg_addr++)&0x7F done as per the
		SPI communication protocol specified in the data sheet
		*/
		index = stringpos * BME280_ADDRESS_INDEX;
		array[index] = (reg_addr) & SPI_WRITE;
		SSIDataPut(SSI0_BASE, array[index]);
		while(SSIBusy(SSI0_BASE))
		{
		} 
		array[index + BME280_DATA_INDEX] = *(reg_data + stringpos);
		SSIDataPut(SSI0_BASE, array[index + BME280_DATA_INDEX]);
		while(SSIBusy(SSI0_BASE))
		{
		} 
	}
	
	/*
	 * CS = '1'
	 */
	CS = 0x02;
	

}// void sendSPI(unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)

void cleanFIFO(void)
{
	unsigned long ulTmp;
	do 
	{
		SSIDataGetNonBlocking(SSI0_BASE, &ulTmp);
	} while (HWREG(SSI0_BASE + SSI_O_SR) & 0x04);
	
}// void cleanFIFO(void);

tBoolean statusSPI(unsigned long ulBase)
{

//*****************************************************************************
//
//! Determines whether the SSI transmitter is busy or not.
//!
//! \param ulBase is the base address of the SSI port.
//!
//! This function allows the caller to determine whether all transmitted bytes
//! have cleared the transmitter hardware.  If \b false is returned, then the
//! transmit FIFO is empty and all bits of the last transmitted word have left
//! the hardware shift register.
//!
//! \return Returns \b true if the SSI is transmitting or \b false if all
//! transmissions are complete.
//
//*****************************************************************************
	
	return SSIBusy(ulBase);

}
