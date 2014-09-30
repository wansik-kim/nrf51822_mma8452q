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
#include "ds1624.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "synaptics_touchpad.h"
#include "simple_uart.h"
#include "boards/nrf6310.h"

#define DS1624_ADDRESS      0x07    /**< Bits [2:0] describing how pins A2, A1 and A0 are wired. */
#define TOUCHPAD_ADDRESS    0x20    /**< Touchpad TWI address in bits [6:0]. */

#define ERROR_PIN                18 /**< gpio pin number to show error if loopback is enabled. */
#define MAX_TEST_DATA_BYTES      (15U)   /**< max number of test bytes to be used for tx and rx. */

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

//////////////////////////////////


bool touchpad_read_register2(uint8_t device_address, uint8_t register_address, uint8_t *value)
{
  bool transfer_succeeded = true;
  transfer_succeeded &= twi_master_transfer(device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
  if (transfer_succeeded) 
  {
    transfer_succeeded &= twi_master_transfer(device_address | TWI_READ_BIT, value, 1, TWI_ISSUE_STOP);
  }
  return transfer_succeeded;
}  
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Delay for touchpad power up 
    nrf_delay_ms(400);

		uint8_t data;
    bool accel_init_succeeded;
    bool ds1624_init_succeeded;
    bool m_conversion_in_progress = false;

//    nrf_gpio_port_dir_set(NRF_GPIO_PORT_SELECT_PORT1, NRF_GPIO_PORT_DIR_OUTPUT);
//		nrf_gpio_cfg_output(3);
//		nrf_gpio_cfg_output(4);
		nrf_gpio_cfg_output	(LED_0);
		nrf_gpio_cfg_output	(LED_1);
		nrf_gpio_pin_set(LED_0);
//		nrf_gpio_pin_set(LED_1);
//		nrf_gpio_pin_clear(LED_0);
//		nrf_gpio_pin_clear(LED_1);
	
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
		if(!accel_init(MMA8452_ADDRESS << 1)){
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
		nrf_delay_ms(500);

		accel_init_succeeded = touchpad_read_register2(MMA8452_ADDRESS << 1,WHO_AM_I,&data);
		nrf_delay_ms(4);
		simple_uart_putstring((const uint8_t *) "\r\ndata : ");
		simple_uart_put(data);
		
		if(accel_init_succeeded){
			nrf_gpio_pin_clear(LED_1);
			nrf_gpio_pin_set(LED_0);
			simple_uart_putstring((const uint8_t *) "\r\nMMA8452Q is online...");
		}
		else{
			nrf_gpio_pin_clear(LED_0);
			nrf_gpio_pin_clear(LED_1);
			simple_uart_putstring((const uint8_t *) "\r\nMMA8452Q is offline...");
		}
}
/** @} */
