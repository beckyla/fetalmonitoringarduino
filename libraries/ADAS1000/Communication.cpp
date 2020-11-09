/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 570
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"
#include <SPI.h>

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - idle state for SPI clock is low.
 *	                          0x1 - idle state for SPI clock is high.
 * @param clockPha - SPI clock phase (0 or 1).
 *                   Example: 0x0 - data is latched on the leading edge of SPI
 *                                  clock and data changes on trailing edge.
 *                            0x1 - data is latched on the trailing edge of SPI
 *                                  clock and data changes on the leading edge.
 *
 * @return 0 - Initialization failed, 1 - Initialization succeeded.
*******************************************************************************/
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockPha)
{
	//Start the SPI library
	SPI.begin();

	int dataMode;

	//Choose dataMode based on clock polarity and phase
	if (clockPol && clockPha) { dataMode = SPI_MODE0;}
	else if (!clockPol && clockPha) { dataMode = SPI_MODE1;}
	else if (clockPol && !clockPha) { dataMode = SPI_MODE2;}
	else if(!clockPol && !clockPha) { dataMode = SPI_MODE3;}
	else {	}

	//Check which dataOrder 
	if (lsbFirst) { 
		SPI.beginTransaction(SPISettings(clockFreq, LSBFIRST, dataMode));
	}
	else { 
		SPI.beginTransaction(SPISettings(clockFreq, MSBFIRST, dataMode));
	}

    return 1;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(unsigned char* data,
						unsigned char bytesNumber)
{
	unsigned char byte = 0;

	//Set the Chip select pin low to signal transmission
	digitalWrite(6, LOW);

	//Write to SPI
	for (unsigned char i = 0; i<bytesNumber; i++)
	{
		SPI.transfer(data[i]);
	}
	
	//Set the Chip select pin to high to terminate transmission
	digitalWrite(6, HIGH);

	return bytesNumber;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - Data represents the read buffer.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char SPI_Read(unsigned char* data,
					   unsigned char bytesNumber)
{
	unsigned char byte = 0;

	//Set the Chip select pin low to signal transmission
	digitalWrite(6, LOW);

	//Read SPI
	for (unsigned char i = 0; i<bytesNumber; i++)
	{
		data[i] = SPI.transfer(0x00);
	}

	//Set the Chip select pin to high to terminate transmission
	digitalWrite(6, HIGH);

	return bytesNumber;
}
