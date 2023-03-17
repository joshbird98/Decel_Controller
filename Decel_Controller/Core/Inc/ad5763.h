/*
 * ad5763.h
 *
 *  Created on: Feb 11, 2023
 *      Author: Josh
 */

#ifndef SRC_AD5763_H_
#define SRC_AD5763_H_

#include "stdint.h"
#include "main.h"

struct ad5763_device {
  uint16_t sync_pin;
  GPIO_TypeDef sync_port;
  SPI_HandleTypeDef spi_handle;
  char dev_name[20];
  char ch1_name[20];
  char ch2_name[20];
  float ch1_offset;
  float ch1_gain;
  float ch1_out_uV;
  float ch1_stage_out_uV;
  float ch2_offset;
  float ch2_gain;
  float ch2_out_uV;
  float ch2_stage_out_uV;
  float temp_C;
};

struct ad5763_device init_ad5763(uint16_t sync_pin, GPIO_TypeDef * sync_port,
		SPI_HandleTypeDef spi_handle, float ch1_offset, float ch1_gain,
		float ch2_offset, float ch2_gain, uint8_t verbose, char *dev_name, char *ch1_name, char *ch2_name);
struct ad5763_device set_stage_uV(struct ad5763_device dac, uint8_t channel, float stage_target_uV, uint8_t verbose);
struct ad5763_device set_output_uV(struct ad5763_device dac, uint8_t channel, float target_uV, uint8_t verbose);
void set_output_bits(struct ad5763_device dac, uint8_t channel, uint16_t bits);

#endif /* SRC_AD5763_H_ */
