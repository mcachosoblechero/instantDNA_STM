#ifndef __PWMSAMPLING_H
#define __PWMSAMPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "instantDNA.h"

/************* PLATFORM VARIABLES ***********/
extern DAC_HandleTypeDef hdac;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
/********************************************/

/************* TIMER VARIABLES ****************/
struct TimerChannelParam {
	__IO uint32_t TicksPeriod_Sample;
	__IO uint32_t TicksHigh_Sample;
	__IO float DutyCycle_Sample;
	__IO float Frequency_Sample;
	__IO uint32_t NumSamples;
	
	__IO char ActiveMeas;
	__IO char SampleAvailable;
	__IO char FirstIgnored;
	__IO char NumIter;
	
	__IO char ValidSample;
};

volatile struct TimerChannelParam TimerCh2;
/**********************************************/

/********** FUNCTIONS **************/
void AcquireData(void);
void ProcessData(void);
/***********************************/


/**********************/
/* PROCESSING METHODS */
/**********************/
void AcquireData(){

	uint32_t TicksHigh;
	uint32_t TicksPeriod;
	
	/****************************/
	/* GO THROUGH EVERY CHANNEL */
	/* CHANNEL 2 - PWM 1				*/
	/****************************/
	if (TimerCh2.SampleAvailable){
		
				/* Get the input capture value */
				TicksHigh = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
				TicksPeriod = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

				if (TicksHigh < TicksPeriod && TimerCh2.FirstIgnored) {
					
					// WINDOW SAMPLING
					TimerCh2.TicksPeriod_Sample += TicksPeriod;
					TimerCh2.TicksHigh_Sample += TicksHigh;
					TimerCh2.NumSamples++;
				}
				else TimerCh2.FirstIgnored = 1;
				
				/* Lower flag */
				TimerCh2.SampleAvailable = 0;
	}
	
}

void ProcessData(){
	
	if (TimerCh2.TicksPeriod_Sample == 0){
		TimerCh2.Frequency_Sample = 1.0;
		if(HAL_GPIO_ReadPin(PWM_GPIO_Port, PWM_Pin) == 0x01) {
			TimerCh2.DutyCycle_Sample = (float)1.0;
		}
		else {
			TimerCh2.DutyCycle_Sample = (float)0.0;
		}
		TimerCh2.ValidSample = 1;
	}
	else {
		TimerCh2.DutyCycle_Sample = (float)TimerCh2.TicksHigh_Sample / (float)TimerCh2.TicksPeriod_Sample;
		TimerCh2.Frequency_Sample = (float)84000000.0 / ((float)TimerCh2.TicksPeriod_Sample / (float)TimerCh2.NumSamples);
		if (TimerCh2.DutyCycle_Sample <= 1 & TimerCh2.DutyCycle_Sample >= 0) TimerCh2.ValidSample = 1;
	}
	
}

/*********************/
/* INTERRUPT METHODS */
/*********************/
/* CAPTURE METHOD		 */
/*********************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	
	if (htim == &htim2) TimerCh2.SampleAvailable = 1;
	
}

/*****************************/
/* PERIOD ELAPSED METHOD		 */
/*****************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == &htim3) TimerCh2.ActiveMeas = 0;
	
}


#endif /* __SAMPLINGPWM_H */
