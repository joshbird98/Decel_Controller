/*
 * max11270.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Josh
 */

#include "max11270.h"
#include "main.h"
#include "misc_funcs.h"

//max11270 stuff
const uint8_t MAX11270_REG_WRITE = 0b11000000;
const uint8_t MAX11270_REG_CTRL1_WRITE = 0b11000010;
const uint8_t MAX11270_REG_CTRL1_READ = 0b11000011;
const uint8_t MAX11270_REG_READ = 0b11000001;
const uint8_t MAX11270_READ_DATA = 0b11001101;
const uint8_t MAX11270_CMD = 0b10000000;
const uint8_t MAX11270_CAL = 0b10100000;

struct max11270_device init_max11270(uint16_t cs_pin, GPIO_TypeDef* cs_port,
		uint16_t rdy_pin, GPIO_TypeDef* rdy_port, uint8_t speed, SPI_HandleTypeDef spi_handle, uint8_t verbose, char *name)
{
    struct max11270_device adc;
	strcpy(adc.name, "MAX11270-");
    strcat(adc.name, name);
	if (verbose > 0)
	{
		print("Initialising ");
		println(adc.name);
	}
    adc.cs_pin = cs_pin;
    adc.cs_port = cs_port;
    adc.rdy_pin = rdy_pin;
    adc.rdy_port = rdy_port;
    adc.spi_handle = spi_handle;
    adc.result_bits = 0;
    adc.result_uV = 0;
    adc.fault = 0;
    if (speed > 0x0F) speed = 0x0F;
    adc.speed = speed;

	// check that device responds, ie read STAT register
	uint16_t status = stat_max11270(&adc, verbose);

    setup_max11270(&adc, verbose);
    //calibrate_max11270(adc, verbose);
    if (verbose > 0)
    {
    	if (status == 14480)
    	{
    		print(adc.name);
    		println(" initialised");
    	}
    	else
    	{
    		print(adc.name);
    		println(" failure, no stat reg readback.");
    	}
    }
    cont_conversion(&adc, verbose, adc.speed);
    return adc;
}

//calibrate MAX11270
void calibrate_max11270(struct max11270_device *adc, uint8_t verbose)
{
	if (verbose > 0)
	{
		print("Calibrating ");
		println(adc->name);
	}
	setup_SPI_max11270(adc);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&adc->spi_handle, (uint8_t *)&MAX11270_CAL, 1, 100);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);

	return_SPI_max11270(adc);

	HAL_Delay (200);   /* Insert delay 200 ms */
	if (verbose > 0)
	{
		print(adc->name);
		println(" calibrated");
	}
}

//read from MAX11270
void read_max11270(struct max11270_device *adc, struct max6225_device *vref, uint8_t verbose)
{
	uint8_t ADC_buffer[3] = {0,0,0};
	if (verbose > 1)
	{
		print("Reading sample from ");
		println(adc->name);
	}
	setup_SPI_max11270(adc);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&adc->spi_handle, (uint8_t *)&MAX11270_READ_DATA, 1, 100);
	HAL_SPI_Receive(&adc->spi_handle, ADC_buffer, 3, 100);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);
	uint32_t sample = ((uint32_t)ADC_buffer[0] << 16) | ((uint32_t)ADC_buffer[1] << 8) | ((uint32_t)ADC_buffer[2]);
	adc->result_bits = sample;
	adc->result_uV = max11270_bits_to_uV(adc->result_bits, *vref);

	return_SPI_max11270(adc);

	if (verbose > 1)
	{
		print("Sample read from ");
	    print(adc->name);
	    print(" = ");
	    printuint32_t(sample, 1);
	}
}

//setup MAX11270
//update in future for customisable settings?
void setup_max11270(struct max11270_device *adc, uint8_t verbose)
{
	if (verbose > 0)
	{
		print("Setting up ");
		println(adc->name);
	}
	setup_SPI_max11270(adc);

	// write CTRL1 register, continuous mode, unipolar reference, internal clock
	uint8_t tx_data[2] = {0,0};
	tx_data[0] = MAX11270_REG_CTRL1_WRITE;
	tx_data[1] = 0b00001101;
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&adc->spi_handle, tx_data, 2, 100);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);

	// readback CTRL1 register, check correct
	uint8_t CTRL1_reg[1] = {0};
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&adc->spi_handle, (uint8_t *)&MAX11270_REG_CTRL1_READ, 1, 100);
	HAL_SPI_Receive(&adc->spi_handle, CTRL1_reg, 1, 100);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);
	uint8_t readback = (uint8_t)CTRL1_reg[0];
	if (readback != tx_data[1]) adc->fault = 1;

	return_SPI_max11270(adc);

	if (verbose > 0)
	{
		print("CTRL1: ");
		printuint8_t(readback, 1);
		print(adc->name);
		if (adc->fault == 0)
			println(" is setup");
		else
			println(" did not respond to setup request.");
	}
}

float max11270_bits_to_uV(uint32_t bits, struct max6225_device vref)
{
	float uV = ((float)bits * (vref.output / 15764879.0));
	return uV;
}

//readback stat register
uint16_t stat_max11270(struct max11270_device *adc, uint8_t verbose)
{

	setup_SPI_max11270(adc);
	uint8_t STAT_reg[2] = {0, 0};

	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&adc->spi_handle, (uint8_t *)&MAX11270_REG_READ, 1, 100);
	HAL_SPI_Receive(&adc->spi_handle, STAT_reg, 2, 100);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);
	uint16_t val = ((uint16_t)STAT_reg[0] << 8) | ((uint16_t)STAT_reg[1]);
	if ((val & 0B1111001100001100) == 0B0011000000000000) adc->fault = 0;
	else adc->fault = 1;

	return_SPI_max11270(adc);

	if (verbose == 1)
	{
		printuint16_t(val, 1);
	}
	if (verbose == 2)
	{
		print(adc->name);
		print(" status value is ");
		printuint16_t(val, 1);
		if (adc->fault == 0) println("No faults detected.");
		else println("Fault detected.");
	}
	return val;
}

uint8_t check_available_max11270(struct max11270_device *adc)
{
	return !HAL_GPIO_ReadPin(adc->rdy_port, adc->rdy_pin); //note inverted!
}


void cont_conversion(struct max11270_device *adc, uint8_t verbose, uint8_t speed)
{
	// valid speeds are 0 to 0xF
	if ((speed >= 0) && (speed <= 0x0F))
	{
		adc->speed = (speed & 0x0F);
	}

	setup_SPI_max11270(adc);

	uint8_t value[1] = {(0B10000000 | (adc->speed & 0B00001111))};
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&adc->spi_handle, value, 1, 100);
	HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET);

	return_SPI_max11270(adc);

	if (verbose > 0)
	{
		print(adc->name);
		print(" starting conversions at data rate: ");
		printuint8_t(adc->speed, 1);
		println("");
	}
}

void setup_SPI_max11270(struct max11270_device *adc)
{
	adc->spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	adc->spi_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	adc->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	if (HAL_SPI_Init(&adc->spi_handle) != HAL_OK)
	{
		Error_Handler();
	}
}

void return_SPI_max11270(struct max11270_device *adc)
{
	adc->spi_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
	adc->spi_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
	adc->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	if (HAL_SPI_Init(&adc->spi_handle) != HAL_OK)
	{
		Error_Handler();
	}
}

