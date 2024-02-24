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

/* src includes*/
#include "middleware.h"
#include "utilities.h"

/* free RTOS template code utilised and found at: https://www.freertos.org/Hardware-independent-RTOS-example.html*/

/*-----------------------------------------------------------*/
/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY   ( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainQUEUE_SEND_PERIOD_MS            pdMS_TO_TICKS( 200 )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainSOFTWARE_TIMER_PERIOD_MS        pdMS_TO_TICKS( 1000 )

/* The number of items the queue can hold.  This is 1 as the receive task
has a higher priority than the send task, so will remove items as they are added,
meaning the send task should always find the queue empty. */
#define mainQUEUE_LENGTH                    ( 1 )
/*-----------------------------------------------------------*/

// Middleware Setup
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );
static void vExampleTimerCallback( TimerHandle_t xTimer );
static void prvEventSemaphoreTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by the queue send and queue receive tasks. */
static QueueHandle_t xQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static SemaphoreHandle_t xEventSemaphore = NULL;

/* The counters used by the various examples.  The usage is described in the
 * comments at the top of this file.
 */
static volatile uint32_t ulCountOfTimerCallbackExecutions = 0;
static volatile uint32_t ulCountOfItemsReceivedOnQueue = 0;
static volatile uint32_t ulCountOfReceivedSemaphores = 0;

/*-----------------------------------------------------------*/

// Defined Tasks for traffic control
static void prvTrafficFlowAdjustmentTask( void *pvParameters );
static void prvTrafficGeneratorTask ( void *pvParameters );
static void prvTrafficLightStateTask ( void *pvParameters );
static void prvSystemDisplayTask ( void *pvParameters );

/*-----------------------------------------------------------*/

// Queue Handler Definitions
xQueueHandle xFlowAdjustmentQueue = 0; //queue used to pass FlowState between tasks
xQueueHandle xSystemStateQueue = 0; //queue used to pass SystemState struct between tasks to share info of light state and traffic state

/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	/* Setup Queues */
	xFlowAdjustmentQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(FlowState));
	xSystemStateQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(SystemState));

	/* create traffic control tasks */
	xTaskCreate( prvTrafficFlowAdjustmentTask, "Traffic Flow Adjustment", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL);
	xTaskCreate( prvTrafficGeneratorTask, "Traffic Generator", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL);
	xTaskCreate( prvTrafficLightStateTask, "Traffic Light State", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL);
	xTaskCreate( prvSystemDisplayTask, "System Display Update", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL);

	/* Initialize Default System State */
	SystemState initialSystemState;
	initialSystemState.lightState = GREEN;
	initialSystemState.trafficState = 0x00;

	/* Initialize Default Traffic Flow State */
	FlowState initialFlowState; 
	
	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	while(1){}

	return 0;
}

/*-----------------------------------------------------------*/

/* Task Used to Read From Potentiometer and Send Flow State to Other tasks (one to generate traffic and one to control light changing) */
static void prvTrafficFlowAdjustmentTask( void *pvParameters )
{
	uint16_t currentPotValue;
	FlowState currentFlow;

	while(1){
	vTaskDelay(400); // Delay for 400 ticks or 200ms

	xQueueReceive(xFlowAdjustmentQueue, &currentPotValue, portMAX_DELAY); // Receive value from queue to clear queue

	currentPotValue = readPot(); // Read the potentiometer value

    // Set currentFlow based on Potentiometer Value
	if(currentPotValue <= POT_MAX/4) currentFlow = LIGHT_TRAFFIC;
	else if(currentPotValue <= POT_MAX/2) currentFlow = MODERATE_TRAFFIC;
	else if(currentPotValue <= 3*POT_MAX/4) currentFlow = HIGH_TRAFFIC;
	else currentFlow = HEAVY_TRAFFIC;

	xQueueSend(xFlowAdjustmentQueue, &currentFlow, 100); // Send flow rate value to queue
	}
}

/*-----------------------------------------------------------*/

/* Task Used to Generate Traffic and Update SystemState.TrafficState to then pass to SystemDisplayTask */
static void prvTrafficGeneratorTask ( void *pvParameters )
{
	SystemState systemStateToUpdate; // SystemState to update
	FlowState currentFlow; // current flow 
	srand((unsigned int)xTaskGetTickCount()); //seed random number generator with current number of ticks


	while(1){
		vTaskDelay(500); //Execute every 250ms

		xQueueRecieve(xFlowAdjustmentQueue, &currentFlow, 100); // Receive current Flow State
		xQueueRecieve(xFlowAdjustmentQueue, &systemStateToUpdate, 100); // Receive current System State to update

		if(systemStateToUpdate.lightState == GREEN)
		{


		}


	}

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

static void prvTrafficLightStateTask ( void *pvParameters )
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

static void prvSystemDisplayTask ( void *pvParameters )
{
	SystemState systemStateToWrite; //initialize system state to recieve values from queue

	while(1){
		xQueueReceive(xSystemStateQueue, &systemStateToWrite, portMAX_DELAY); //recieve updated System State from queue

		updateSystem(&systemStateToWrite); //update system (update lights and traffic)

		xQueueSend(xSystemStateQueue, &systemStateToWrite, 0); // send boardstate back onto queue
	}
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	NVIC_SetPriorityGrouping( 0 ); // Ensure all priority bits are assigned as preemption priority bits.
	GPIOInit(); // GPIO Initialization
	ADCInit(); // ADC Initialization
	SystemCoreClockUpdate(); // Update the system with the new clock frequency
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
