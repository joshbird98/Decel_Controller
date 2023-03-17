/*
 * spi_bare.h
 *
 *  Created on: March 17, 2023
 *      Author: Josh
 */

#ifndef SRC_SPI_BARE_H_
#define SRC_SPI_BARE_H_

#include "stdint.h"
#include "main.h"

void SPI_Receive (uint8_t *data, int size, SPI_TypeDef *spi);

#endif /* SRC_SPI_BARE_H_ */
