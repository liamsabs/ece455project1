
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
//static void Manager_Task( void *pvParameters );

xQueueHandle xQueue_handle = 0;

void adc_initialize( void );
void traffic_lights( float traffic_flow );
void status_toggle( void );
uint16_t adc_convert( void );

void delay( uint16_t delay_duration );

unsigned char digits[10] = { 0x3f, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

/*-----------------------------------------------------------*/

int main(void)
{


	adc_initialize();
	//led_init();

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();
	GPIO_Setup();

	//float traffic_flow = 2;

	// ADC Testing Function
//	int count = 0;
//	while(1){
//		if(count > 4800000){
//			printf("ADC Value is: %u \n", (unsigned int)ADC_GetConversionValue(ADC1));
//			count = 0;
//		}
//		count++;
//	}


//	xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//
//	/* Start the tasks and timer running. */
//	vTaskStartScheduler();


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
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */


//	// Disable the PLL
//	RCC->CR &= ~(RCC_CR_PLLON);
//	// Wait for the PLL to unlock
//	while (!( RCC->CR & RCC_CR_PLLRDY ));

//	RCC->CFGR 	|= RCC_CFGR_PLLSRC;

	// Configure the PLL for 48-MHz system clock
	RCC->CFGR = 0x00280000;
	// Enable the PLL
	RCC->CR |= RCC_CR_PLLON;

	while (!( RCC->CR & RCC_CR_PLLRDY ));
	RCC->CFGR 	|= RCC_CFGR_SW_PLL;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))

//	// Wait for the PLL to lock
//	while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );
//	// Switch the processor to the PLL clock source
//	RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;

	// Update the system with the new clock frequency
	SystemCoreClockUpdate();


	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR (Defined as TIM2CLK)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

}

/*-----------------------------------------------------------*/

static void GPIO_Setup( void )
{
	// Enable GPIOA & GPIOB Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// Setting up PC0 as output (Red traffic light)
	GPIOC->MODER |= GPIO_MODER_MODER0_0; // Output
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0); //Pull-up Disabled

	// Setting up PC1 as output (Amber traffic light)
	GPIOC->MODER |= GPIO_MODER_MODER1_0; // Output
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1); //Pull-up Disabled

	// Setting up PC2 as output (Green traffic light)
	GPIOC->MODER |= GPIO_MODER_MODER2_0; // Output
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2); //Pull-up Disabled

	// Setting up PC3 as input (Potentiometer)
	GPIOC->MODER |= GPIO_MODER_MODER3; //analog mode
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR3); // Pull-up Disabled

	// Setting up PC6 as output (Shift register data)
	GPIOC->MODER |= GPIO_MODER_MODER6_0; // Output
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6); //Pull-up Disabled

	// Setting up PC8 as output (Shift register clock)
	GPIOC->MODER |= GPIO_MODER_MODER8_0; // Output
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8); // Pull-up Disabled

	// Setting up PC9 as output (Shift register reset)
	GPIOC->MODER |= GPIO_MODER_MODER9_0; // Output
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9); // Pull-up Disabled
}

/*-----------------------------------------------------------*/

void adc_initialize( void )
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enables the ADC1 Clock
	ADC_InitTypeDef ADCInit_Structure; // Initialize ADC Init Struct
	ADC_StructInit(&ADCInit_Structure); // Provide Default Configurations
	ADC_Init(ADC1, &ADCInit_Structure); // Enables and configures the prescaler, unsure if these are the correct values
	ADC_Cmd(ADC1, ENABLE); // Activates the ADC peripheral
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_144Cycles); // Sets it to channel 0
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
