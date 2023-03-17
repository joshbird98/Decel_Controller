/*
 * mcp33131.c
 *
 *  Created on: Feb 10, 2023
 *      Author: Josh
 */

#include "mcp33131.h"
#include "main.h"
#include "misc_funcs.h"
#include "max6225.h"
#include "spi_bare.h"

struct mcp33131_device init_mcp33131(uint16_t cs_pin, GPIO_TypeDef * cs_port, SPI_TypeDef * spi_handle, uint8_t verbose, char *name)
{
	struct mcp33131_device adc;
	strcpy(adc.name, "MCP33131-");
	strcat(adc.name, name);
	if (verbose > 0)
	{
		print("Initialising ");
		println(adc.name);
	}
	adc.cs_pin = cs_pin;
	adc.cs_port = cs_port;
	adc.spi_handle = spi_handle;
	adc.result_bits = 0;
	adc.result_uV = 0;
	adc.cal_start_time = 0;
	adc.cal_active = 0;
	adc.last_clock = DWT->CYCCNT;
	adc.min_clk_cycles = (uint16_t)(CNV_TIME * ((float)HAL_RCC_GetHCLKFreq() / 1000000000.0));
	calibrate_mcp33131(&adc, verbose);
	while (check_available_mcp33131(&adc) == 0) {}
	if (verbose > 0)
	{
		print(adc.name);
		println(" calibrated and initialised\n");
	}
	return adc;
}

void calibrate_mcp33131(struct mcp33131_device *adc, uint8_t verbose)
{
	if (verbose > 0)
	{
		print("Calibrating ");
		println(adc->name);
	}

	// Calibration inititiated by 1024 cycles ie 128 bytes worth
	uint8_t buffer[128] = {[0 ... 127] = 0xFF};

	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin))<<16; 				// set CS LOW
	SPI_Transmit(buffer, 128, adc->spi_handle);					// transmit 128 dummy bytes to start calibration
	adc->cs_port->BSRR |= (1<<(adc->cs_pin))<<16; 				// reset CS HIGH

	// calibration can now take up to 650ms, so device should not be read until complete
	adc->cal_start_time = HAL_GetTick();
	adc->cal_active = 1;
}

uint8_t check_available_mcp33131(struct mcp33131_device *adc)
{
	//check if calibration is occuring
	if (adc->cal_active == 1)
	{
		uint32_t cal_time_elapsed = HAL_GetTick() - adc->cal_start_time;
		if (cal_time_elapsed < CAL_TIME) return 0;
	}
	//check if enough time has elapsed
	uint32_t new_clock = DWT->CYCCNT;
	uint32_t clocks_elapsed = new_clock - adc->last_clock;
	if (clocks_elapsed >= adc->min_clk_cycles) return 1;
	else return 0;
}

// should only be called if check_available has returned true, or if user is sure enough time has passed
void read_mcp33131(struct mcp33131_device *adc, struct max6225_device *vref, uint8_t verbose)
{
	adc->cal_active = 0;
	if (verbose > 1)
	{
		println("Reading sample from ");
		print(adc->name);
	}

	// this does seem a bit buggy, and sometimes only gets 15 bits...
	uint8_t buffer[2] = {0,0};


	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin))<<16; 				// set CS LOW
	SPI_Receive(buffer, 2, adc->spi_handle);					// receive 2 bytes
	adc->cs_port->BSRR |= (1<<(adc->cs_pin))<<16; 				// reset CS HIGH

	adc->last_clock = DWT->CYCCNT;
	adc->result_bits = ((uint16_t)buffer[0] << 8 ) | ((uint16_t) buffer[1]);
	adc->result_uV = mcp33131_bits_to_uV(adc->result_bits, *vref);

	if (verbose > 1)
	{
		print("Sample read from ");
		print(adc->name);
		print(" = ");
		printuint16_t(adc->result_uV, 1);
	}
}

float mcp33131_bits_to_uV(uint16_t bits, struct max6225_device vref)
{
	return (((float)bits) * vref.output) / 63727.56;
}

