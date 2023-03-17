/*
 * board.h
 *
 *  Created on: Feb 23, 2023
 *      Author: Josh
 */

#ifndef SRC_BOARD_H_
#define SRC_BOARD_H_

#include "stdint.h"
#include "max11270.h"
#include "mcp33131.h"
#include "main.h"


struct hv_feedback_divider {
	float r1_nominal;
	float r2_nominal;
	float r1_nominal_temp;
	float r2_nominal_temp;
	float r1_temp_co;
	float r2_temp_co;
	float r1;
	float r2;
	float ratio;
};

struct board {
	uint8_t hv_enable;					// SAFETY CRITICAL, in series with interlocks, and then HV supplies
	uint8_t hv_enable_readback;			// will read high if (hv_enable == 1) AND (all interlocks connected)
	int32_t target_decel_voltage;		// requested voltage for the decel voltage (voltage from HP020)
	float decel_voltage_slow;				// measured voltage of the decel voltage (from the HP020 output), in mV
	float decel_voltage_fast;				// measured voltage of the decel voltage (from the HP020 output), in mV
	int32_t target_decel_coarse_voltage;		// ideal voltage output of DAC HV1 coarse *stage*
	int32_t target_decel_coarse_fine;			// ideal voltage output of DAC HV1 fine *stage*
	int32_t focus_voltage;				// requested voltage for the focus electrode (UV_A output)
	int32_t injection_voltage;			// requested voltage for the injection electrode (UV_B output
	uint8_t error;						// flag to be set on unexpected conditions
	struct hv_feedback_divider divider; // complicated structure for handling the HV feedback
};

struct command {
	uint8_t valid;
	char type[40];
	char object[40];
	float value;
};


struct board init_board(uint8_t verbose);

struct hv_feedback_divider init_divider(uint8_t verbose);

float temp_comp_divider(struct hv_feedback_divider, float temp_K);

void updateDecelReadingSlow(struct board *decel_board, struct max11270_device *adc, uint8_t verbose);

void updateDecelReadingFast(struct board *decel_board, struct mcp33131_device *adc, uint8_t verbose);

#endif /* SRC_BOARD_H_ */
