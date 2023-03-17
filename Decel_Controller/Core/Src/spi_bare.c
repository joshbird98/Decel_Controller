/*
 * spi_bare.c , see https://controllerstech.com/spi-using-registers-in-stm32/
 *
 *  Created on: March 17, 2023
 *      Author: Josh
 *
 *  Both SPI1 and SPI2 must be controlled, therefore each function must be passed this
 */


#include "spi_bare.h"
#include "main.h"

//SPI_TypeDef *spi should be either SPI1 or SPI2
void SPI_Receive (uint8_t *data, int size, SPI_TypeDef *spi)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/

	while (size)
	{
		while (((spi->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		spi->DR = 0;  // send dummy data
		while (!((spi->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	    *data++ = (spi->DR);
		size--;
	}

}

void SPI_Transmit (uint8_t *data, int size, SPI_TypeDef *spi)
{

	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/

	int i=0;
	while (i<size)
	{
	   while (!((spi->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   spi->DR = data[i];  // load the data into the Data Register
	   i++;
	}


	/*During discontinuous communications, there is a 2 APB clock period delay between the
	write operation to the SPI_DR register and BSY bit setting. As a consequence it is
	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
	data.
	*/
	while (!((spi->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((spi->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = spi->DR;
	temp = spi->SR;

}

/************** STEPS TO FOLLOW *****************
1. Enable SPI clock
2. Configure the Control Register 1
3. Configure the CR2
************************************************/
void SPIConfig (SPI_TypeDef *spi, uint8_t mode, uint32_t max_freq)
{
	if (spi == SPI1) RCC->APB2ENR |= (1<<12);  // Enable SPI1 CLock, max 84MHz
	if (spi == SPI2) RCC->APB1ENR |= (1<<14);  // Enable SPI2 Clock, max 42MHz

	SPIMode(spi, mode);		    	// determine CPHA and CPOL
	SPISpeed(spi, max_freq); 		// determine optimal baud rate divider to attain SCLK speed up to max_freq

	spi->CR1 |= (1<<2);  			// Master Mode
	spi->CR1 &= ~(1<<7);  			// LSBFIRST = 0, MSB first
	spi->CR1 |= (1<<8) | (1<<9);  	// SSM=1, SSi=1 -> Software Slave Management
	spi->CR1 &= ~(1<<10);  			// RXONLY = 0, full-duplex
	spi->CR1 &= ~(1<<11);  			// DFF=0, 8 bit data
	spi->CR2 = 0;
}

// sets the CPOL and CPHA values
void SPIMode(SPI_TypeDef *spi, uint8_t mode)
{
	mode &= 0x3;
	if (mode == 0)      spi->CR1 |= (0<<0)|(0<<1);
	else if (mode == 1) spi->CR1 |= (0<<0)|(1<<1);
	else if (mode == 2) spi->CR1 |= (1<<0)|(0<<1);
	else if (mode == 3) spi->CR1 |= (1<<0)|(1<<1);
}

// finds the settings that will produce the highest SCLK freq <= max_freq
// slow because involves HAL call to find
void SPISpeed(spi, max_freq)
{
	uint32_t clk = 0;
	if (spi == SPI1) clk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos];
	if (spi == SPI2) clk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2)>> RCC_CFGR_PPRE2_Pos];

	// available SPI speeds are clk/2, clk/4, clk/8 etc...
	uint8_t br_div = 0;
	while (((clk << (br_div + 1)) > max_freq) && (br_div < 8))
	{
		br_div += 1;
	}
	SPIBRDiv(spi, br_div);
}

// For SPI1, SPI clk = 84MHz / (2 ^ (br_div + 1)
// For SPI2, SPI clk = 42MHz / (2 ^ (br_div + 1)
void SPIBRDiv(SPI_TypeDef *spi, uint8_t br_div)
{
	spi->CR1 |= ((br_div & 0x07) << 3);
}


void SPI_Enable (SPI_TypeDef *spi)
{
	spi->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_Disable (SPI_TypeDef *spi)
{
	spi->CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}

void CS_Enable (void)
{
	GPIOA->BSRR |= (1<<9)<<16;
}

void CS_Disable (void)
{
	GPIOA->BSRR |= (1<<9);
}


void GPIOConfig (void)
{
	RCC->AHB1ENR |= (1<<0);  // Enable GPIO Clock

	GPIOA->MODER |= (2<<10)|(2<<12)|(2<<14)|(1<<18);  // Alternate functions for PA5, PA6, PA7 and Output for PA9

	GPIOA->OSPEEDR |= (3<<10)|(3<<12)|(3<<14)|(3<<18);  // HIGH Speed for PA5, PA6, PA7, PA9

	GPIOA->AFR[0] |= (5<<20)|(5<<24)|(5<<28);   // AF5(SPI1) for PA5, PA6, PA7
}
