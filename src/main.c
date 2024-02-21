
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );
static void GPIO_Setup( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Manager_Task( void *pvParameters );

xQueueHandle xQueue_handle = 0;

void led_init( void );
void led_data( unsigned char data );
void adc_initialize( void );
void status_toggle( void );
uint16_t adc_convert( void );

void Delay( void );

unsigned char digits[10] = { 0x3f, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

/*-----------------------------------------------------------*/

int main(void)
{


	adc_initialize ();
	//led_init();

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();
	GPIO_Setup();

	int count = 0;
		while(1){
			if(count > 4800000){
				printf("ADC Value is: %u \n", (unsigned int)ADC_GetConversionValue(ADC1));
				count = 0;
			}
			count++;
		}


	xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}


/*-----------------------------------------------------------*/

static void Manager_Task( void *pvParameters )
{
//
// Display mode range 1 to 3
//
	uint16_t mode = 1;

	uint16_t counter = 0;
	uint16_t delay = 0;

	uint16_t adc_data = 0;

	while(1)
	{
		adc_data = adc_convert() / 409;
		if ( adc_data >= 10 )
		{
			adc_data = 9;
		}


		if ( mode == 1 )
		{
			status_toggle();
			delay = 250;
			led_data( digits[counter] );
		}
		else if ( mode == 2 )
		{
			delay = 250;
			led_data( digits[adc_data] );
		}
		else
		{
			delay = (100*adc_data) + 100;
			led_data( digits[counter] );
		}

		counter = counter + 1;
		if ( counter >= 10 )
		{
			counter = 0;
		}


		vTaskDelay( delay );
	}
}

/*-----------------------------------------------------------*/

void adc_initialize( void ){

	ADC_InitTypeDef ADCInit_Structure;
	ADC_StructInit(&ADCInit_Structure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enables the ADC1 Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Enables the clock for the ADC1 GPIOA
	ADC_Init(ADC1, &ADCInit_Structure); // Enables and configures the prescaler, unsure if these are the correct values
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
	ADC_Cmd(ADC1, ENABLE); // Activates the ADC peripheral


	// Can then use _GetMultiModeConversionValue() to get the value

}

/*-----------------------------------------------------------*/

//void led_init( void ){
//
//}

/*-----------------------------------------------------------*/

//void led_data( unsigned char data ){
//
//}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */


	// Disable the PLL
	RCC->CR &= ~(RCC_CR_PLLON);
	// Wait for the PLL to unlock
	while (!( RCC->CR & RCC_CR_PLLRDY ));


	// Configure the PLL for 48-MHz system clock
	RCC->CFGR = 0x00280000;
	// Enable the PLL
	RCC->CR |= RCC_CR_PLLON;
	// Wait for the PLL to lock
	while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );
	// Switch the processor to the PLL clock source
	RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
	// Update the system with the new clock frequency
	SystemCoreClockUpdate();


	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR (Defined as TIM2CLK)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

}

static void GPIO_Setup( void )
{
	// Enable GPIOA Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// Enable GPIOC Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// Setting up PA0 as analog high-speed input (For ADC)
	// Relevant register: GPIOA->MODER (Defined as PA0Mode)
	GPIOA->MODER |= GPIO_MODER_MODER0;
	GPIOA->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1); // Setting it to 0x11 for analog mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);


	// Setting up PC0 as output (Red traffic light)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER0;
	GPIOC->MODER &= ~(GPIO_MODER_MODER0); // Setting it to output mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	// Setting up PC1 as output (Amber traffic light)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER1;
	GPIOC->MODER &= ~(GPIO_MODER_MODER1); // Setting it to output mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	// Setting up PC2 as output (Green traffic light)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER2;
	GPIOC->MODER &= ~(GPIO_MODER_MODER2); // Setting it to output mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2);

	// Setting up PC3 as input (Potentiometer)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER3;
	GPIOC->MODER &= ~(GPIO_MODER_MODER3); // Setting it to Input mode <--- might not be correct!
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	// Setting up PC6 as output (Shift register data)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER6;
	GPIOC->MODER &= ~(GPIO_MODER_MODER6); // Setting it to output mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

	// Setting up PC7 as output (Shift register clock)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER7;
	GPIOC->MODER &= ~(GPIO_MODER_MODER7); // Setting it to output mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

	// Setting up PC8 as output (Shift register reset)
	// Relevant register: GPIOC->MODER (Defined as PC0Mode)
	GPIOC->MODER |= GPIO_MODER_MODER8;
	GPIOC->MODER &= ~(GPIO_MODER_MODER8); // Setting it to output mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
}
