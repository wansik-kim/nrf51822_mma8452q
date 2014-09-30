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
		
//		MMA8452Standby();
		
//		while(1){
//				int accelCount[3];  // Stores the 12-bit signed value
//				readAccelData(accelCount);  // Read the x/y/z adc values
//			
//				simple_uart_putstring((const uint8_t *) "\r\naccelCount[0] :   ");
//				simple_uart_putstring((const uint8_t *) accelCount[0]);
//			
//				nrf_delay_ms(500);
//		}
		
//    ds1624_init_succeeded       = ds1624_init(DS1624_ADDRESS);

//    // If both failed to initialized, halt here.
//    if (!touchpad_init_succeeded && !ds1624_init_succeeded)
//    {
//        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x5F);
//        while (true) 
//        {
//            // Do nothing.
//        }
//    }

//    while(true)
//    {
//        if (ds1624_init_succeeded)
//        {
//            if (!m_conversion_in_progress) 
//            {
//                m_conversion_in_progress = ds1624_start_temp_conversion();
//            }
//            else
//            {
//            // Succeeded.
//                if (ds1624_is_temp_conversion_done())
//                {
//                    m_conversion_in_progress = false;
//                    int8_t temperature;
//                    int8_t temperature_fraction;
//      
//                    if (ds1624_temp_read(&temperature, &temperature_fraction))
//                    {
//                        nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0x7F);
//                        nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)temperature);
//                    }
//                }
//            }
//        }

//        if (touchpad_init_succeeded)
//        {       
//            uint8_t touchpad_button_status; // read button status.
//            if (touchpad_read_register(TOUCHPAD_BUTTON_STATUS, &touchpad_button_status))
//            {
//                // There's a active low button on the side of the touchpad, check the state and light up a LED if it's pressed.
//                nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0x80);
//                if (!(touchpad_button_status & 0x01))
//                {
//                    nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT1, 0x80);
//                }
//            }
//        }
//    }
}
/** @} */
