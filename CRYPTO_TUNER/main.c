//*****************************************************************************
//
// single_ended.c - Example demonstrating how to configure the ADC for
//                  single ended operation.
//
// Copyright (c) 2010-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "adc.h"
#include "adc.c"
#include "gpio.h"
#include "gpio.c"
#include "pin_map.h"
#include "sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/uart.c"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "utils/ustdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/interrupt.c"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.c"
#include "driverlib/uart.h"
#include "arm_math.h"
#include "timer.h"
#include "timer.c"
#include "driverlib/cpu.h"
#include "driverlib/cpu.c"


#include "fpu.h"
#include "fpu.c"


#define FFT_SIZE 4096
#define sampleRate 40000
//*****************************************************************************
//
//! \addtogroup adc_examples_list
//! <h1>Single Ended ADC (single_ended)</h1>
//!
//! This example shows how to setup ADC0 as a single ended input and take a
//! single sample on AIN0/PE7.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC0 peripheral
//! - GPIO Port E peripheral (for AIN0 pin)
//! - AIN0 - PE7
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of the
//! ADC.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.
//
//*****************************************************************************

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************

	/////////ADC STUFF///////
	uint32_t ui32SysClock;

    uint32_t pui32ADC0Value[1] ;
    int32_t count = 0;
    uint32_t result;
    float32_t fftData[FFT_SIZE*2];
    float32_t fftResult[FFT_SIZE*2];
    uint32_t temp[FFT_SIZE*2];
    ///////ARM STUFF//////////
            		float32_t wavFreqHz;
            		arm_status status;
            	    arm_cfft_radix2_instance_f32 fft;
            	    float32_t maxValue;
            	    uint32_t testIndex = 0;
            	    uint32_t ifftFlag = 0;
            	    uint32_t doBitReverse = 1;
            	    //uint32_t sampleRate = 40000;//SysCtlClockGet();
            	    //UARTprintf("\n%4d\r", sampleRate);
            	    float32_t hzPerBin = 2 * ((float32_t)sampleRate/(float32_t)FFT_SIZE);
            	    uint32_t complexBuffSize = FFT_SIZE * 2;


            	    uint32_t *temp32;             /* Point to data for 32 bit samples */

            	    int32_t   temp32Data;

            	    int_fast32_t i32IntegerPart;
            	    int_fast32_t i32FractionPart;
            	    int32_t string = 1;
            	    int button0pressed = 0;
            	    int button1pressed = 0;




void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, 16000000);
}












//*****************************************************************************
//
// Configure ADC0 for a single-ended input and a single sample.  Once the
// sample is ready, an interrupt flag will be set.  Using a polling method,
// the data will be read then displayed on the console via UART0.
//
//*****************************************************************************
int
main(void)
{
	FPUEnable();
	FPULazyStackingEnable();

	temp32Data = 0;

    //
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    //





    //
    // Set the clocking to run at 20 MHz (200 MHz / 10) using the PLL.  When
    // using the ADC, you must either use the PLL or supply a 16 MHz clock
    // source.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	                                             SYSCTL_OSC_MAIN |
	                                             SYSCTL_USE_PLL |
	                                             SYSCTL_CFG_VCO_480), 120000000);

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for ADC operation.
    //
    InitConsole();

    //
    // Display the setup on the console.
    //
    UARTprintf("\n\nGUITAR TUNER ->\nSTART AT LOW E\n");










    	    //InitADC();

    /*ADC*/
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	// ADC0 module enable
        //SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);	// 1.000.000 samples per second
        ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

        ADCSequenceDisable(ADC0_BASE, 3);			//Disable ADC Sequencer 0
        //Sequencer will be triggered by one of the general-purpose timers.
        ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
        ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH6| ADC_CTL_IE | ADC_CTL_END);
        ADCSequenceEnable(ADC0_BASE, 3);


    /*Timer*/
    //TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000);//Timer timer couldn't be higher than sample rate
    //Enable the timer’s ADC output trigger
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    // Enable the timer and start conversion process
    // TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);


    ADCIntEnable(ADC0_BASE, 3);
    	IntEnable(INT_ADC0SS3);
    	IntMasterEnable();


    /////////DRIVE CONFIG/////////////
    	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    		    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)));

    		    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6); //direction
    		    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7); //steps

    /////////BUTTON CONFIG////////////
    		    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    		    	    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)));

    		    	    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    		    	    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);





        while(1){
        	button0pressed = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
        	SysCtlDelay(100);

        }




///////////////////////////////////////////////////////////////////////////////////////////////////////












}

void ADC0IntHandler(void){

					if(count < complexBuffSize){


        		    	//i++;
        		        //
        		        // Trigger the ADC conversion.
        		        //
        		        //ADCProcessorTrigger(ADC0_BASE, 3);

        		        //
        		        // Wait for conversion to be completed.
        		        //
        		        //while(!ADCIntStatus(ADC0_BASE, 3, false))
        		        //{
        		        //}

        		        //
        		        // Clear the ADC interrupt flag.
        		        //
        		        ADCIntClear(ADC0_BASE, 3);

        		        //
        		        // Read ADC Value.
        		        //
        		        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);




        		        	if(count % 2 == 0){
        		            	 fftData[count] = (float32_t) pui32ADC0Value[0];
        		            	 count++;

        		            	 }
        		            else
        		            	{
        		            	fftData[count] = 0.0000000;
        		            	count++;

        		            	}
        		        }
					else{
						if(string == 1){
						IntMasterDisable();

						status = ARM_MATH_SUCCESS;

						/* Set instance for Real FFT */
						status = arm_cfft_radix2_init_f32(&fft, FFT_SIZE, ifftFlag, doBitReverse);

						/* Perform Real FFT on fftData */
						arm_cfft_radix2_f32(&fft, fftData);

						/* Populate FFT bins */
						arm_cmplx_mag_f32(fftData, fftResult, FFT_SIZE);

						/* Zero out non-audible, low-frequency noise from FFT Results. */
						fftResult[0] = 0.000000000000000;

						/* Find max bin and location of max (first half of bins as this is the only valid section) */
						arm_max_f32(fftResult, FFT_SIZE, &maxValue, &testIndex);


						wavFreqHz = testIndex * hzPerBin *0.07779379086012135641633225614232; //;;;;/40.69988f

						if(fabs(wavFreqHz - 82.41) > 50){
							UARTprintf("\nKeep Plucking Low E!!! ->\n");
						}
						else{

						if(fabs(wavFreqHz - 82.41)<= 1.0){
							int delayCount = 0;
							UARTprintf("\nIn Tune!! Move to the Next String!! ->\n");
							while(delayCount != 5){

								delayCount++;
								SysCtlDelay(12000000);
							}
							string++;
						}
						else if(wavFreqHz - 82.41 > 2.0){
							GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
							SysCtlDelay(6000);
							int i = 0;
							for(i; i < 10; i++){
								GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
								SysCtlDelay(12000);
								GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
								SysCtlDelay(12000);
								UARTprintf("\nSHARP!!! ->\n");

							}
						}
						else if(wavFreqHz - 82.41 > 1.0){
							GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
							SysCtlDelay(6000);
							int i = 0;
								for(i; i < 3; i++){
									GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
									SysCtlDelay(12000);
									GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
									SysCtlDelay(12000);
									UARTprintf("\nSHARP!!! ->\n");

								}
						}
						else if(wavFreqHz - 82.41 < -2.0){
							GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
							SysCtlDelay(6000);
							int i = 0;
							for(i; i < 10; i++){
								GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
								SysCtlDelay(12000);
								GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
								SysCtlDelay(12000);
								UARTprintf("\nFLAT!!! ->\n");

							}
						}
						else if(wavFreqHz - 82.41 < -1.0){
							GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
							SysCtlDelay(6000);
							int i = 0;
							for(i; i < 3; i++){
								GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
								SysCtlDelay(12000);
								GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
								SysCtlDelay(12000);
								UARTprintf("\nFLAT!!! ->\n");

							}
						}



						}


						i32IntegerPart = (int32_t)wavFreqHz;
						i32FractionPart = (int32_t)(wavFreqHz * 1000.0000f);
						i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
						if(i32FractionPart < 0)
						{
							i32FractionPart *= -1;
						}
						UARTprintf("Frequency %5d.%04d\n", i32IntegerPart, i32FractionPart);
						count = 0;
						IntMasterEnable();
						}
						else if(string == 2){
												IntMasterDisable();

												status = ARM_MATH_SUCCESS;

												/* Set instance for Real FFT */
												status = arm_cfft_radix2_init_f32(&fft, FFT_SIZE, ifftFlag, doBitReverse);

												/* Perform Real FFT on fftData */
												arm_cfft_radix2_f32(&fft, fftData);

												/* Populate FFT bins */
												arm_cmplx_mag_f32(fftData, fftResult, FFT_SIZE);

												/* Zero out non-audible, low-frequency noise from FFT Results. */
												fftResult[0] = 0.000000000000000;

												/* Find max bin and location of max (first half of bins as this is the only valid section) */
												arm_max_f32(fftResult, FFT_SIZE, &maxValue, &testIndex);


												wavFreqHz = testIndex * hzPerBin*0.07823473978058711616080795149446; //;;;;/40.69988f
												if(fabs(wavFreqHz - 110) > 50){
													UARTprintf("\nKeep Plucking the A!!! ->\n");
												}
												else{
												if(fabs(wavFreqHz - 110)<= 1.0){
																			int delayCount = 0;
																			UARTprintf("\nIn Tune!! Move to the Next String!! ->\n");
																			while(delayCount != 5){

																				delayCount++;
																				SysCtlDelay(12000000);
																			}
																			string++;
																		}
																		else if(wavFreqHz - 110 > 2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}
																			UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 110 > 1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																				while(i < 3){
																					GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																					SysCtlDelay(12000);
																					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																					SysCtlDelay(12000);
																					i++;

																				}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 110 < -2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
																		else if(wavFreqHz - 110 < -1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
												}

												i32IntegerPart = (int32_t)wavFreqHz;
												i32FractionPart = (int32_t)(wavFreqHz * 1000.0000f);
												i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
												if(i32FractionPart < 0)
												{
													i32FractionPart *= -1;
												}
												UARTprintf("Frequency %5d.%04d\n", i32IntegerPart, i32FractionPart);
												count = 0;
												IntMasterEnable();
												}
						else if(string == 3){
												IntMasterDisable();

												status = ARM_MATH_SUCCESS;

												/* Set instance for Real FFT */
												status = arm_cfft_radix2_init_f32(&fft, FFT_SIZE, ifftFlag, doBitReverse);

												/* Perform Real FFT on fftData */
												arm_cfft_radix2_f32(&fft, fftData);

												/* Populate FFT bins */
												arm_cmplx_mag_f32(fftData, fftResult, FFT_SIZE);

												/* Zero out non-audible, low-frequency noise from FFT Results. */
												fftResult[0] = 0.000000000000000;

												/* Find max bin and location of max (first half of bins as this is the only valid section) */
												arm_max_f32(fftResult, FFT_SIZE, &maxValue, &testIndex);


												wavFreqHz = testIndex * hzPerBin*0.0784; //;;;;/40.69988f
												if(fabs(wavFreqHz - 146.83) > 50){
														UARTprintf("\nKeep Plucking the D!!! ->\n");
												}
												else{
												if(fabs(wavFreqHz - 146.83)<= 1.0){
																			int delayCount = 0;
																			UARTprintf("\nIn Tune!! Move to the Next String!! ->\n");
																			while(delayCount != 5){

																				delayCount++;
																				SysCtlDelay(12000000);
																			}
																			string++;
																		}
																		else if(wavFreqHz - 146.83 > 2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 146.83 > 1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																					GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																					SysCtlDelay(12000);
																					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																					SysCtlDelay(12000);
																					i++;


																				}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 146.83 < -2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
																		else if(wavFreqHz - 146.83 < -1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
												}

												i32IntegerPart = (int32_t)wavFreqHz;
												i32FractionPart = (int32_t)(wavFreqHz * 1000.0000f);
												i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
												if(i32FractionPart < 0)
												{
													i32FractionPart *= -1;
												}
												UARTprintf("Frequency %5d.%04d\n", i32IntegerPart, i32FractionPart);
												count = 0;
												IntMasterEnable();
												}
						if(string == 4){
												IntMasterDisable();

												status = ARM_MATH_SUCCESS;

												/* Set instance for Real FFT */
												status = arm_cfft_radix2_init_f32(&fft, FFT_SIZE, ifftFlag, doBitReverse);

												/* Perform Real FFT on fftData */
												arm_cfft_radix2_f32(&fft, fftData);

												/* Populate FFT bins */
												arm_cmplx_mag_f32(fftData, fftResult, FFT_SIZE);

												/* Zero out non-audible, low-frequency noise from FFT Results. */
												fftResult[0] = 0.000000000000000;

												/* Find max bin and location of max (first half of bins as this is the only valid section) */
												arm_max_f32(fftResult, FFT_SIZE, &maxValue, &testIndex);


												wavFreqHz = testIndex * hzPerBin*0.07780701407207335168917241164944; //;;;;/40.69988f
												if(fabs(wavFreqHz - 196) > 50){
														UARTprintf("\nKeep Plucking the G!!! ->\n");
												}

												else{
												if(fabs(wavFreqHz - 196)<= 1.0){
																			int delayCount = 0;
																			UARTprintf("\nIn Tune!! Move to the Next String!! ->\n");
																			while(delayCount != 5){

																				delayCount++;
																				SysCtlDelay(12000000);
																			}
																			string++;

																		}
																		else if(wavFreqHz - 196 > 2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 196 > 1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																					GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																					SysCtlDelay(12000);
																					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																					SysCtlDelay(12000);
																					i++;


																				}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 196 < -2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
																		else if(wavFreqHz - 196 < -1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
												}


												i32IntegerPart = (int32_t)wavFreqHz;
												i32FractionPart = (int32_t)(wavFreqHz * 1000.0000f);
												i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
												if(i32FractionPart < 0)
												{
													i32FractionPart *= -1;
												}
												UARTprintf("Frequency %5d.%04d\n", i32IntegerPart, i32FractionPart);
												count = 0;
												IntMasterEnable();
												}
						if(string == 5){
												IntMasterDisable();

												status = ARM_MATH_SUCCESS;
												//UARTprintf("\n\nCheck ->\n");
												/* Set instance for Real FFT */
												status = arm_cfft_radix2_init_f32(&fft, FFT_SIZE, ifftFlag, doBitReverse);

												/* Perform Real FFT on fftData */
												arm_cfft_radix2_f32(&fft, fftData);

												/* Populate FFT bins */
												arm_cmplx_mag_f32(fftData, fftResult, FFT_SIZE);

												/* Zero out non-audible, low-frequency noise from FFT Results. */
												fftResult[0] = 0.000000000000000;

												/* Find max bin and location of max (first half of bins as this is the only valid section) */
												arm_max_f32(fftResult, FFT_SIZE, &maxValue, &testIndex);


												wavFreqHz = testIndex * hzPerBin*0.07806558659714383619096574463097; //;;;;/40.69988f
												if(fabs(wavFreqHz - 246.94) > 50){
													UARTprintf("\nKeep Plucking the B!!! ->\n");
												}
												else{
												if(fabs(wavFreqHz - 246.94)<= 1.0){
																			int delayCount = 0;
																			UARTprintf("\nIn Tune!! Move to the Next String!! ->\n");
																			while(delayCount != 5){

																				delayCount++;
																				SysCtlDelay(12000000);
																			}
																			string++;
																		}
																		else if(wavFreqHz - 246.94 > 2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;

																			}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 246.94 > 1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																					GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																					SysCtlDelay(12000);
																					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																					SysCtlDelay(12000);
																					i++;

																				}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 246.94 < -2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;

																			}UARTprintf("\nFLAT!!! ->\n");
																		}
																		else if(wavFreqHz - 246.94 < -1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;

																			}UARTprintf("\nFLAT!!! ->\n");
																		}
												}

												i32IntegerPart = (int32_t)wavFreqHz;
												i32FractionPart = (int32_t)(wavFreqHz * 1000.0000f);
												i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
												if(i32FractionPart < 0)
												{
													i32FractionPart *= -1;
												}
												UARTprintf("Frequency %5d.%04d\n", i32IntegerPart, i32FractionPart);
												count = 0;
												IntMasterEnable();
												}
						if(string == 6){
												IntMasterDisable();

												status = ARM_MATH_SUCCESS;
												//UARTprintf("\n\nCheck ->\n");
												/* Set instance for Real FFT */
												status = arm_cfft_radix2_init_f32(&fft, FFT_SIZE, ifftFlag, doBitReverse);

												/* Perform Real FFT on fftData */
												arm_cfft_radix2_f32(&fft, fftData);

												/* Populate FFT bins */
												arm_cmplx_mag_f32(fftData, fftResult, FFT_SIZE);

												/* Zero out non-audible, low-frequency noise from FFT Results. */
												fftResult[0] = 0.000000000000000;

												/* Find max bin and location of max (first half of bins as this is the only valid section) */
												arm_max_f32(fftResult, FFT_SIZE, &maxValue, &testIndex);


												wavFreqHz = testIndex * hzPerBin*0.07823473978058711616080795149446; //;;;;/40.69988f
												if(fabs(wavFreqHz - 329.63) > 50){
													UARTprintf("\nKeep Plucking the High E!!! ->\n");
												}
												else{
												if(fabs(wavFreqHz - 329.63)<= 1.0){
													UARTprintf("\nIn Tune!! Reset to Start Again!!  ->");
																			while(1){

																				SysCtlDelay(12000000);
																			}
																		}
																		else if(wavFreqHz - 329.63 > 2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;

																			}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 329.63 > 1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																					GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																					SysCtlDelay(12000);
																					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																					SysCtlDelay(12000);
																					i++;

																				}UARTprintf("\nSHARP!!! ->\n");
																		}
																		else if(wavFreqHz - 329.63 < -2.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 10){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;


																			}UARTprintf("\nFLAT!!! ->\n");
																		}
																		else if(wavFreqHz - 329.63 < -1.0){
																			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
																			SysCtlDelay(6000);
																			int i = 0;
																			while(i < 3){
																				GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
																				SysCtlDelay(12000);
																				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
																				SysCtlDelay(12000);
																				i++;

																			}UARTprintf("\nFLAT!!! ->\n");
																		}
												}

												i32IntegerPart = (int32_t)wavFreqHz;
												i32FractionPart = (int32_t)(wavFreqHz * 1000.0000f);
												i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
												if(i32FractionPart < 0)
												{
													i32FractionPart *= -1;
												}
												UARTprintf("Frequency %5d.%04d\n", i32IntegerPart, i32FractionPart);
												count = 0;
												IntMasterEnable();
												}
					}










        	}
