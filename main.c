/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *
 * This file contains the source code for a the flow rate measurement.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "bsp.h" 
#include "app_pwm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAMPLES_IN_BUFFER 1														
#define HEATER_PIN NRF_SAADC_INPUT_AIN0               // analog pin 0 - sampling v_heater
#define TEMP_DIFF_PIN NRF_SAADC_INPUT_AIN2            // analog pin 2 - sampling v_diff (temp diff)


APP_PWM_INSTANCE(PWM1,1);  									  				// creating PWM instance using Timer 1

static 				nrf_saadc_value_t  m_buffer_pool[2][SAMPLES_IN_BUFFER];
static float 	v_heater;
static float 	adc_sample; 								//the saadc sample trigger returns a value to this variable
static volatile bool 	m_sampling = false; //flag true DURING sampling - prevent channel reassignment
static volatile bool	ready_flag;         // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}


/* saadc_callback:
Once sample task triggered, averages samples in buffer, then converts into a voltage, avg_sample_v
This value passed to global adc_sample, where main loop can access it once sampling done
*/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event) 
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        int avg_sample = 0;
        float avg_sample_v;
        int i;
        ret_code_t err_code;
		
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)	
        {
              avg_sample += p_event->data.done.p_buffer[i]; // take N samples in a row
        }
        avg_sample /= i; // average all the samples out
				
				avg_sample_v = avg_sample*0.000879; //convert from sample to voltage (gain = 1/6, ref = 0.6, res = 10)
				/* in sdk_config.h, #define NRFX_SAADC_CONFIG_RESOLUTION 2, so 12-bit ADC resolution
														#define NRFX_SAADC_CONFIG_OVERSAMPLE 2, so 4x oversampling
														avg_sample = [avg_sample_v] * GAIN/REFERENCE * 2^(RESOLUTION)
														for GAIN = 1/6 and REFERENCE = 0.6V (see config)*/
				adc_sample = avg_sample_v;
				m_sampling = false;
    }
}

void saadc_init_heater(void)
{
		//initialising saadc channel for heater
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	
    channel_config.gain = NRF_SAADC_GAIN1_6;	
		channel_config.burst = NRF_SAADC_BURST_ENABLED; //OVERSAMPLING TO LIMIT NOISE

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

void saadc_init_temp(void)
{
		//initialising saadc channel for temp sensors
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
	
    channel_config.gain = NRF_SAADC_GAIN1_6;
		channel_config.burst = NRF_SAADC_BURST_ENABLED; //OVERSAMPLING TO LIMIT NOISE

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_trigger(void)
{
  ret_code_t err_code;
	m_sampling = true;
	//Event handler is called immediately after conversion is finished.
	err_code = nrf_drv_saadc_sample(); // Check error
	APP_ERROR_CHECK(err_code);
}


uint16_t duty_cycle_to_ticks(float x)
{
	uint16_t y; //has to be uint16_t
	if (x==0){
		y = 0;
	}
	else {
		x = (x*1600)*0.01; //if 5kHz duty cycle, full duty cycle = 1600,
		y = ((x)>=0?(int)((x)+0.5):(int)((x)-0.5)); //rounding to uint16, integer value
	}
	return y;
}

float max(float x, float y)
{
	float z;
	z = (x<y)? (y) :(x);
	return z;
}

float min(float x, float y)
{
	float z;
	z = (x<y)? (x) :(y);
	return z;
}

float LUT1(float R_RTD_DIFF)
{
	float T_DIFF_OUT;
	int RTD_DIFF_LUT[] = {0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60};
	
	//LUT using CVD values
	float TEMP_DIFF_LUT[] = {0,	1.0236,	2.0475,	3.0718,	4.0963,	5.1212,	
														6.1464,	7.1718,	8.1976,	9.2237,	10.2502,	11.2769,	
															12.3039,	13.3313,	14.3589,	15.3869};	
	    for (int i=0; i<16; i++)
			{

            if (R_RTD_DIFF>RTD_DIFF_LUT[i])
						{
                ; //placeholder
						}
            else
						{
                //stop and do lin.interpol
                T_DIFF_OUT = TEMP_DIFF_LUT[i-1]+(TEMP_DIFF_LUT[i]-TEMP_DIFF_LUT[i-1])*((R_RTD_DIFF-RTD_DIFF_LUT[i-1])/(RTD_DIFF_LUT[i]-RTD_DIFF_LUT[i-1]));
								return T_DIFF_OUT;
						}
			 }
			return 0;
}

float LUT2(float T_DIFF)
{
	float flowrate;
	int TEMP_DIFF_LUT[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
		
	//LUT using quadratic fit values
	float FLOWRATE_LUT[] = {0.263,	1.055,	1.876,	2.73,	3.621,	4.554,	5.537,	6.578,	7.689,	
														8.885,	10.191,	11.643,	13.305,	15.31,	18.041,	21.5};
	
	    for (int i=0; i<16; i++)
			{

            if (T_DIFF>TEMP_DIFF_LUT[i])
						{
                ; //placeholder
						}
            else
						{
                //stop and do lin.interpol
                flowrate = FLOWRATE_LUT[i-1]+(FLOWRATE_LUT[i]-FLOWRATE_LUT[i-1])*((T_DIFF-TEMP_DIFF_LUT[i-1])/(TEMP_DIFF_LUT[i]-TEMP_DIFF_LUT[i-1]));
								return flowrate;
						}
			 }
			return 0;
}

int main(void)
{
	//----SETUP
	
	//Step 1: init uart/NRF
	uint32_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	//-------------------------
	
	//Step 2: init PWM

	ret_code_t err_code_pwm;

	/* 1-channel PWM, 5kHz (200us), output on pin25. 500000L for 2Hz*/
	app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(200L, ARDUINO_13_PIN); //period in us
	err_code_pwm = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
	APP_ERROR_CHECK(err_code_pwm);
	app_pwm_enable(&PWM1);
	//-----------------------

	//Step 3: Setting initial variables: TO CUSTOMISE

	float p_target = 200; //mW
	float Kp = 0.25;
	float temp_sample_freq = 1; //2Hz
	float v_heater, i_heater, p_heater, p_error; 
	float v_temp_diff, temp_diff, flow_rate;
	float Vdd = 3.6; 
	float r_sense = 0.5; //TODO Check
	float duty_cycle; //as a percentage
	duty_cycle = 97; //set initial PWM duty cycle as a percentage
	float r_diff; //this is the measured r_diff between the up and downstream RTDs, converted from v_temp_diff
	float gain_1 = 3;
	float gain_2 = 25; 
	float I_excite = 0.0006; 
	float v_offset = 0.5; 

	uint32_t sampling_period_ms = 500;  //setting sampling frequency 
	uint8_t control_time_period = 10;		//setting control frequency
	uint8_t control_loop_no = sampling_period_ms/control_time_period; //the control loop runs until a sample is taken at the sampling time period
	//-----------------------
	
	//Step 4 [optional]: wait x amount of time for flow to develop
	NRF_LOG_INFO("START\r\n");
	NRF_LOG_INFO("Waiting for flow to develop...\r\n");
	for (uint8_t j=0; j<5; ++j)
	{
		NRF_LOG_INFO(".");
		nrf_delay_ms(1000);
	}

	
	//Step 6: Start PWM, based on resistor at 0 degrees Celsius
	NRF_LOG_INFO("Starting PWM...\r\n");
	ready_flag = false;
	uint16_t set_duty_cycle_ticks = duty_cycle_to_ticks(duty_cycle); //convert % duty cycle to ticks
	while (app_pwm_channel_duty_ticks_set(&PWM1, 0, set_duty_cycle_ticks) == NRF_ERROR_BUSY);
	while (!ready_flag); 
	APP_ERROR_CHECK(app_pwm_channel_duty_ticks_set(&PWM1, 1, set_duty_cycle_ticks)); 

	
	//Step 7: Waiting for heater to get to steady state (15s)
	NRF_LOG_INFO("Heating up... \r\n");
	NRF_LOG_FLUSH();
	for (uint8_t j=0; j<5; ++j)
	{
		NRF_LOG_INFO(".");
		nrf_delay_ms(1000);
	}

	
	NRF_LOG_INFO("Flow Rate Measurement Started...\r\n");
	
	while(true){
		saadc_init_heater();
		for(uint8_t i=0; i<control_loop_no; ++i) //loops for 50*10ms = 0.5s = sampling time period
		{
			saadc_sampling_trigger();
			v_heater = adc_sample; 
			i_heater = (Vdd-v_heater)/r_sense; 
			p_heater = v_heater*i_heater*1000; //in mW
			
			//calc new duty cycle
			p_error = p_target - p_heater;
			duty_cycle += (100*(p_error/p_target))*Kp;
			
			duty_cycle = max(0, duty_cycle); //if duty cycle ends up negative, set to 0
			duty_cycle = min(100, duty_cycle); //if duty cycle >100, then ceil at 100
			
			//update duty cycle and pwm task
			ready_flag = false;
			set_duty_cycle_ticks = duty_cycle_to_ticks(duty_cycle); //convert % duty cycle to ticks
			while (app_pwm_channel_duty_ticks_set(&PWM1, 0, set_duty_cycle_ticks) == NRF_ERROR_BUSY);
			while (!ready_flag);
			APP_ERROR_CHECK(app_pwm_channel_duty_ticks_set(&PWM1, 1, set_duty_cycle_ticks));
			
			nrf_delay_ms(control_time_period); //10ms period - sets control loop frequency
		}
		while (m_sampling == true) {}; //wait for heater control to finish using saadc resource
    nrf_drv_saadc_uninit();        //unitialise so that temp_diff can use saadc resource
			
		saadc_init_temp(); //now intialise channel to read temp_diff voltage
		saadc_sampling_trigger();
		while (m_sampling == true) {};	
		nrf_drv_saadc_uninit();	

		v_temp_diff = adc_sample;
		NRF_LOG_INFO("Measured v_diff (V):" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(v_temp_diff));
		NRF_LOG_FLUSH();
		
		//convert v_temp_diff to a resistance and therefore a temp difference
		r_diff = (v_temp_diff-v_offset)*(1/(gain_1*gain_2))*(1/I_excite);
		
		temp_diff = LUT1(r_diff);
		flow_rate = LUT2(temp_diff);
		NRF_LOG_INFO("Flow Rate = " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(flow_rate));
		NRF_LOG_FLUSH();
		while(true){}; //to remove for continuous operation
	}
	


}


/** @} */
