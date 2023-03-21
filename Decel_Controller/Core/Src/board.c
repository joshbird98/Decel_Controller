/*
 * hv_feedback.c
 *
 *  Created on: Feb 23, 2023
 *      Author: Josh
 */

#include "board.h"
#include "main.h"
#include "misc_funcs.h"

struct board init_board(uint8_t verbose)
{
	struct board decel_board;
	decel_board.error = 0;
	decel_board.hv_enable = 0;
	decel_board.hv_enable_readback = HAL_GPIO_ReadPin(HV_ENABLE_RETURN_GPIO_Port, HV_ENABLE_RETURN_Pin);
	decel_board.divider = init_divider(verbose);
	decel_board.target_decel_voltage = 0;		// requested voltage for the decel voltage (voltage from HP020)
	decel_board.decel_voltage_slow = 0;				// measured voltage of the decel voltage (from the HP020 output)
	decel_board.decel_voltage_fast = 0;				// measured voltage of the decel voltage (from the HP020 output)
	decel_board.target_decel_coarse_voltage = 0;		// ideal voltage output of DAC HV1 coarse *stage*
	decel_board.target_decel_coarse_fine = 0;			// ideal voltage output of DAC HV1 fine *stage*
	decel_board.focus_voltage = 0;				// requested voltage for the focus electrode (UV_A output)
	decel_board.injection_voltage = 0;			// requested voltage for the injection electrode (UV_B output
	return decel_board;
}


struct hv_feedback_divider init_divider(uint8_t verbose)
{
	if (verbose > 0) println("Initialising High-Voltage Feedback Divider");
    struct hv_feedback_divider divider;
    divider.r1_nominal = 375000000.0;
    divider.r2_nominal = 39000.0;
    divider.r1_nominal_temp = 300;
    divider.r2_nominal_temp = 300;
    divider.r1_temp_co = 0;
    divider.r2_temp_co = 0;
    divider.r1 = divider.r1_nominal;
    divider.r2 = divider.r2_nominal;
    divider.ratio = divider.r1 / divider.r2;
    if (verbose > 0) println("High-Voltage Feedback Divider initialised");
    return divider;
}

float temp_comp_divider(struct hv_feedback_divider divider, float temp_K)
{
	divider.r1 = divider.r1_nominal + (divider.r1_temp_co * (temp_K - divider.r1_nominal_temp));
	divider.r2 = divider.r2_nominal + (divider.r2_temp_co * (temp_K - divider.r2_nominal_temp));
	divider.ratio = divider.r1 / divider.r2;
	return divider.ratio;
}

void updateDecelReadingSlow(struct board *decel_board, struct max11270_device *adc, uint8_t verbose)
{
	decel_board->decel_voltage_slow = (adc->result_uV / 1000000.0) * (decel_board->divider.ratio + 1);
	if (verbose > 0)
	{
		print("ADC measures ");
		printfloat(adc->result_uV / 1000000.0, 0);
		print("[V]\n\rDivider ratio: ");
		printfloat(decel_board->divider.ratio, 1);
		print("Decel Voltage is ");
		printfloat(decel_board->decel_voltage_slow, 0);
		println("V");
	}
}


void updateDecelReadingFast(struct board *decel_board, struct mcp33131_device *adc, uint8_t verbose)
{
	decel_board->decel_voltage_fast = (adc->result_uV / 1000000.0) * (decel_board->divider.ratio + 1);
	if (verbose > 0)
	{
		print("ADC measures ");
		printfloat(adc->result_uV / 1000000.0, 0);
		print("[V]\n\rDivider ratio: ");
		printfloat(decel_board->divider.ratio, 1);
		print("Decel Voltage is ");
		printfloat(decel_board->decel_voltage_fast, 0);
		println("V");
	}
}
