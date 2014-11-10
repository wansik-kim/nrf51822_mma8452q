/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
*
* @defgroup twi_master_example_main main.c
* @{
* @ingroup twi_master_example
*
* @brief TWI Master Example Application main file.
*
* This file contains the source code for a sample application using TWI and external sensors. 
*
*/

#include "twi_master.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "simple_uart.h"
#include "boards/nrf6310.h"

//////////////////////////////////
#define BUTTON_0       16
#define BUTTON_1       17

#define LED_0          18
#define LED_1          19
//////////////////////////////////
//Accelerometer
// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D			  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A

#define GSCALE 8
const uint8_t dataRate = 4; // 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56

bool state = false, prev_state = false;
int prev_num, num;
int jcnt = 0;
int max_val = 30, min_val = 13;
//////////////////////////////////


bool read_register(uint8_t device_address, uint8_t register_address, uint8_t *value)
{
  bool transfer_succeeded = true;
  transfer_succeeded &= twi_master_transfer(device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
  if (transfer_succeeded) 
  {
    transfer_succeeded &= twi_master_transfer(device_address | TWI_READ_BIT, value, 1, TWI_ISSUE_STOP);
  }
  return transfer_succeeded;
}  

bool read_registers(uint8_t device_address, uint8_t register_address, int size, uint8_t *value)
{
  bool transfer_succeeded = true;
  transfer_succeeded &= twi_master_transfer(device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
  if (transfer_succeeded) 
  {
    transfer_succeeded &= twi_master_transfer(device_address | TWI_READ_BIT, value, size, TWI_ISSUE_STOP);
  }
  return transfer_succeeded;
}  

bool write_register(uint8_t device_address, uint8_t register_address, const uint8_t value)
{
	uint8_t w2_data[2];
	
	w2_data[0] = register_address;
	w2_data[1] = value;
	twi_master_transfer(device_address, w2_data, 2, TWI_ISSUE_STOP);
  return true;
}

void readAccelData(short int *destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
	bool init = true;
	
  init &= read_registers(MMA8452_ADDRESS << 1,OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array
//	ACC_getdata(rawData);
	nrf_delay_ms(1);
	
	char c[30];
	sprintf(c,"%d",rawData[0]);
	simple_uart_putstring((const uint8_t *) "\r\n");
	simple_uart_putstring((const uint8_t *) c);
	sprintf(c,"%d",rawData[1]);
	simple_uart_putstring((const uint8_t *) "   ");
	simple_uart_putstring((const uint8_t *) c);
	sprintf(c,"%d",rawData[2]);
	simple_uart_putstring((const uint8_t *) "   ");
	simple_uart_putstring((const uint8_t *) c);
	sprintf(c,"%d",rawData[3]);
	simple_uart_putstring((const uint8_t *) "   ");
	simple_uart_putstring((const uint8_t *) c);
	sprintf(c,"%d",rawData[4]);
	simple_uart_putstring((const uint8_t *) "   ");
	simple_uart_putstring((const uint8_t *) c);
	sprintf(c,"%d",rawData[5]);
	simple_uart_putstring((const uint8_t *) "   ");
	simple_uart_putstring((const uint8_t *) c);
	
  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    short int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
		
    gCount  = gCount >> 4; //The registers are left align, here we right align the 12-bit integer
//    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {  
				gCount = ~gCount + 1;
				gCount *= -1;  // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

void parse(int y, int z){
		int sum;
		double dsum, s;
		dsum = y*y+z*z;
		sum = sqrt(dsum);
	
		char c[30];
		sprintf(c,"%d",sum);
		simple_uart_putstring((const uint8_t *) "\r\n");
		simple_uart_putstring((const uint8_t *) c);
		
		if(min_val > sum){
				state = true;                // min
		}
		else if(max_val < sum){
				state = false;               // max
		}
		if(!state && prev_state){
				jcnt++;
		}
		prev_state = state;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
		uint8_t data;
    bool who_am_i;
		bool init = true;
	
		nrf_gpio_cfg_output	(LED_0);
		nrf_gpio_cfg_output	(LED_1);
		nrf_gpio_pin_set(LED_0);
	
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
    simple_uart_putstring((const uint8_t *)" \r\nStart I2C \r\n");
		nrf_delay_ms(500);
		
		if(!twi_master_init()){
        while (true) 
        {
					nrf_gpio_pin_set(LED_0);
					nrf_delay_ms(500);
					nrf_gpio_pin_clear(LED_0);
					nrf_delay_ms(500);
        }
		}
		
		nrf_gpio_pin_set(LED_1);
		nrf_gpio_pin_clear(LED_0);

		who_am_i = read_register(MMA8452_ADDRESS << 1,WHO_AM_I,&data);
		
		
		if(who_am_i){							//data == 0x2A
			nrf_delay_us(100);
			write_register(MMA8452_ADDRESS << 1,XYZ_DATA_CFG, 0x02);
			nrf_delay_us(100);
			write_register(MMA8452_ADDRESS << 1,CTRL_REG1, 0x33);
			nrf_gpio_pin_clear(LED_1);
			nrf_gpio_pin_set(LED_0);
			char c[30];
			sprintf(c,"%d",data);
			simple_uart_putstring((const uint8_t *) "\r\n WHO_AM_I : ");
			simple_uart_putstring((const uint8_t *) c);
			data = '0';
			simple_uart_putstring((const uint8_t *) "\r\nMMA8452Q is online...");
		}
		else{
			nrf_gpio_pin_clear(LED_0);
			nrf_gpio_pin_clear(LED_1);
			simple_uart_putstring((const uint8_t *) "\r\nMMA8452Q is offline...");
		}
		
		short int x[3];


		while(1){
				readAccelData(x);
			
				float accelG[3];  // Stores the real accel value in g's
				
				for (int i = 0 ; i < 3 ; i++)
				{
//					accelG[i] = (float) x[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
					accelG[i] = (float) x[i] / 256;  // get actual g value, this depends on scale being set
					accelG[i] *= 10;
				}
//				int a = accelG[1]*10;
//				int b = accelG[2]*10;
////				parse(a,b);
//			
				char c[30];
				sprintf(c,"%d",x[0]);
				simple_uart_putstring((const uint8_t *) "\r\n");
				simple_uart_putstring((const uint8_t *) c);
				sprintf(c,"%d",x[1]);
				simple_uart_putstring((const uint8_t *) "  ");
				simple_uart_putstring((const uint8_t *) c);
				sprintf(c,"%d",x[2]);
				simple_uart_putstring((const uint8_t *) "  ");
				simple_uart_putstring((const uint8_t *) c);
		}
}
/** @} */
