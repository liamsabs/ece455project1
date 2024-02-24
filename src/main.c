
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

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
//static void Manager_Task( void *pvParameters );

xQueueHandle xQueue_handle = 0;

void adc_initialize( void );
void traffic_lights( float traffic_flow );
void status_toggle( void );
uint16_t adc_convert( void );

void delay( uint16_t delay_duration );

/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	




xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

//	/* Start the tasks and timer running. */
vTaskStartScheduler();
	uint16_t ADC_Value;
	GPIOC->ODR |= GPIO_ODR_ODR_0;
	GPIOC->ODR |= GPIO_ODR_ODR_1;
	GPIOC->ODR |= GPIO_ODR_ODR_2;
	GPIOC->ODR |= GPIO_ODR_ODR_6;
	GPIOC->ODR |= GPIO_ODR_ODR_9;
	for(int i = 0; i < 19; i++){

				GPIO_ToggleBits(GPIOC, GPIO_Pin_8);
				GPIO_ToggleBits(GPIOC, GPIO_Pin_8);
				if((i+2) % 5 == 0) GPIO_ToggleBits(GPIOC, GPIO_Pin_6);
	}

	while(1){

		ADC_SoftwareStartConv(ADC1);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
		{
			ADC_Value = ADC_GetConversionValue(ADC1);
		}
		printf("ADC Value: %d\n", (int)ADC_Value);
	}

	return 0;
}


/*-----------------------------------------------------------*/

//static void Manager_Task( void *pvParameters )
//{
//
//}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	 //http://www.freertos.org/RTOS-Cortex-M3-M4.html 
	NVIC_SetPriorityGrouping( 0 ); // Ensure all priority bits are assigned as preemption priority bits.
	GPIOInit(); // GPIO Initialization
	ADCInit(); // ADC Initialization
	SystemCoreClockUpdate(); // Update the system with the new clock frequency
}

/*-----------------------------------------------------------*/

/*
Updates: Changed traffic_flow to a float as I might be needing decimals, but that loses precision?
Working on finding algorithms to proportionally relate two values
Yellow light needs to be consistent so that's easy
Created a basic delay function for once the actual light durations are figured out

TODO:
- create proportional value algorithm
- Test delay function and dial in the proper times
- Test traffic_lights function with ADC and potentiometer
*/

void traffic_lights( float traffic_flow )
{
	
	GPIOC->ODR |= GPIO_ODR_ODR_0; // example of how to output to the LED
	GPIOC->ODR |= GPIO_ODR_ODR_1;
	GPIOC->ODR |= GPIO_ODR_ODR_2;

	GPIOC->ODR |= GPIO_ODR_ODR_6;
	GPIOC->ODR |= GPIO_ODR_ODR_8;
	GPIOC->ODR |= GPIO_ODR_ODR_9;

	float min = 0;
	float max = 3600;

	float proportion_of_max = traffic_flow/max; // Am attempting to proportionally relate values

	uint16_t red_duration = ; // This needs to be inversely proportional to traffic_flow
	uint16_t yellow_duration = 4;
	uint16_t green_duration = ;

}

/*-----------------------------------------------------------*/

/*
This function is meant to create a delay of a specific amount. 
I will need to test to see exactly how long of a delay this creates, not sure
exactly how best to do that but the goal is to use this for the traffic light 
function and others. Will test this independently in the lab when I have the chance.
*/

void delay( uint16_t delay_duration )
{

	for( uint16_t i = 0; i < 4800000*delay_duration; i++ );

}

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
