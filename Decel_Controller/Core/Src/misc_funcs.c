/*
 * misc_funcs.c
 *
 *  Created on: Feb 10, 2023
 *      Author: Josh
 */

#include "misc_funcs.h"
#include "main.h"
#include "stdint.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <errno.h>
#include "board.h"

void print(const char _out[])
{
  char buf[100];
  sprintf((char*)buf,_out);
  while(CDC_Transmit_FS((uint8_t*)buf, strlen(buf)) == USBD_BUSY) {}
}

void println(const char _out[])
{
  char buf[100];
  sprintf((char*)buf,_out);
  char ending[2] = {'\r', '\n'};
  strncat((char *)buf, (const char *)ending, 2);
  while(CDC_Transmit_FS((uint8_t*)buf, strlen(buf)) == USBD_BUSY) {}
}

void print_uart(const char _out[], UART_HandleTypeDef huart)
{
	uint8_t buf[100];
	sprintf((char*)buf, _out);
	HAL_UART_Transmit(&huart, buf, sizeof(buf), 100);
}

void println_uart(const char _out[], UART_HandleTypeDef huart)
{
	uint8_t buf[100];
	sprintf((char*)buf, _out);
	char ending[2] = {'\r', '\n'};
	strncat((char *)buf, (const char *)ending, 2);
	HAL_UART_Transmit(&huart, buf, sizeof(buf), 100);
}

void print_buffer_ascii(const char _buffer[], uint8_t header)
{
	if (header == 1)
	{
		print("USB data recieved: ");
	}
	char buf[100];
	sprintf((char*)buf, _buffer);
	while(CDC_Transmit_FS((uint8_t*)buf, strlen(buf)) == USBD_BUSY) {}
}

void printuint32_t(uint32_t value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%lu", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {}
}

void printint32_t(int32_t value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%ld", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}


void printint32_t_uart(int32_t value, uint8_t newline, UART_HandleTypeDef huart)
{
  char str[100];
  sprintf(str, "%ld", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  HAL_UART_Transmit(&huart, (uint8_t *)str, sizeof(str), 100);
}

void printuint16_t(uint16_t value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%u", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printint16_t(int16_t value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%d", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printuint8_t(uint8_t value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%u", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printint8_t(int8_t value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%d", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printfloat(float value, uint8_t newline)
{
  char str[100];
  sprintf(str, "%f", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void splash_message(uint8_t verbose)
{
	if (verbose > 0)
	{
		println("~~~ Decel Controller ~~~");
		uint32_t clk_freq_MHz = HAL_RCC_GetHCLKFreq() / 1000000;
		print("Clock Frequency: ");
		printuint32_t(clk_freq_MHz, 0);
		println("MHz");
		print("Verbose level = ");
		printuint8_t(verbose, 1);
		println("");
	}
}

void set_leds(uint8_t led_1_state, uint8_t led_2_state, uint8_t led_3_state)
{
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, led_1_state);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, led_2_state);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, led_3_state);
}

void led_twinkle(uint8_t repeats)
{
	for (uint8_t i = 0; i < repeats; i++)
	{
		set_leds(1,0,0);
		HAL_Delay(100);
		set_leds(0,1,0);
		HAL_Delay(100);
		set_leds(0,0,1);
		HAL_Delay(100);
		set_leds(0,0,0);
		HAL_Delay(100);
	}
}

struct command handle_data(char data[], uint8_t verbose)
{
	struct command cmd;
	cmd.valid = 0;

	//check if data is a command
	if (data[0] == '<')
	{
		char *token;
		char delim[] = ",";
		char end[] = ">";

		if (data[0] == '<')
		{
			token = strtok(data+1, delim);
			strcpy(cmd.type, token);
			if ( token != NULL) token = strtok(NULL, delim);
			strcpy(cmd.object, token);
			if ( token != NULL) token = strtok(NULL, end);

			char *endptr = NULL;
			errno = 0;
			cmd.value = (float)strtol (token, &endptr, 10);
			if ((errno == 0) && (*endptr == '\0'))
			{
				cmd.valid = 1;
			}
		}
	}
	if (verbose > 0)
	{
		if (cmd.valid == 0) println("Unrecognised data input.\n\rNeeds format <cmd,obj,val>");
		else println("Data recognised.");
	}
	return cmd;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	new_uart_data = 1;
	HAL_UART_Receive_IT(&huart3, buffer_uart_rx, 64);
}

void print_binary(uint8_t val)
{
	uint8_t i = 128;
	char str[10] = {0};
	uint8_t dig = 0;
	while (i > 0)
	{
		if ((val & i) > 0) str[dig] = '1';
		else str[dig] = '0';
		i = i / 2;
		dig += 1;
	}
	println(str);
}

