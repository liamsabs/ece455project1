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
static void vTrafficLightTimerCallback ( xTimerHandle timerHandler );
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

// Timer Handler
xTimerHandle xTrafficLightTimerHandler = 0;

/*-------------------------------------------------------------*/

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

	/* Create traffic light timer */
	TimerHandle_t Light_Timer = xTimerCreate
						("Light Timer", 
						pdMS_TO_TICKS(1000), //default value to be changed with specific values
						pdFALSE, // Does not automatically reset
						(void *) 0, // We only have one timer, so this is used for testing and changing the timer
						vTrafficLightTimerCallback); // Fill in the name of the callback function

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
	xQueueReceive(xFlowAdjustmentQueue, &currentPotValue, portMAX_DELAY); // Receive value from queue to clear queue

	currentPotValue = readPot(); // Read the potentiometer value

    // Set currentFlow based on Potentiometer Value
	if(currentPotValue <= POT_MAX/4) currentFlow = LIGHT_TRAFFIC;
	else if(currentPotValue <= POT_MAX/2) currentFlow = MODERATE_TRAFFIC;
	else if(currentPotValue <= 3*POT_MAX/4) currentFlow = HIGH_TRAFFIC;
	else currentFlow = HEAVY_TRAFFIC;

	/*
	The lab manual mentions values being "Directly Proportional" to the Pot value, and I'm just wondering if it expects something like the above,
	or if they want us to have it more directly linked (i.e. _any_ change to the potentiometer will directly affect the traffic rate)
	This would obviously be a lot harder to implement (and has actually been what I've bee struggling to get working the past little bit)
	but thought it would be worth mentioning and considering. I really like the idea of how it's set up currently though, so we'll see.

	If necessary, we can always make more options with smaller fractions, but whatever works works for me!
	*/

	xQueueSend(xFlowAdjustmentQueue, &currentFlow, 100); // Send flow rate value to queue
	vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/*-----------------------------------------------------------*/

/* Task Used to Generate Traffic and Update SystemState.TrafficState to then pass to SystemDisplayTask */
static void prvTrafficGeneratorTask ( void *pvParameters )
{
	SystemState systemStateToUpdate; // SystemState to update
	FlowState currentFlow; // current flow
	srand((unsigned int)xTaskGetTickCount()); //seed random number generator with current number of ticks
	uint32_t tempTrafficState;
	uint32_t beforeIntersectionTemp; //temporary variables to deal with before intersection bits
	uint32_t afterIntersectionTemp; //temporary variables to deal with after intersection bits
	uint16_t randomNumber; //random number generated
	uint16_t probabilityUpperBound; //upper bound for computing task probability
	bool advanceCar; //boolean value on whether or not to add another car to intersection when shifting

	while(1){
		xQueueReceive(xFlowAdjustmentQueue, &currentFlow, 100); // Receive current Flow State
		xQueueReceive(xSystemStateQueue, &systemStateToUpdate, 100); // Receive current System State to update

		//choose the upper bound for the probability calculation
		switch(currentFlow){
			case LIGHT_TRAFFIC:
				probabilityUpperBound = 17;
			break;
			case MODERATE_TRAFFIC:
				probabilityUpperBound = 35;
			break;
			case HIGH_TRAFFIC:
				probabilityUpperBound = 70;
			break;
			case HEAVY_TRAFFIC:
				probabilityUpperBound = 100;
			break;
		}

		randomNumber = rand() % 101; //generate number between 1 and 100
		advanceCar = (randomNumber <= probabilityUpperBound); // If true, advance car is set to true, otherwise false
        tempTrafficState = systemStateToUpdate.trafficState; // tempTrafficState will be used as a temp to manipulate

		switch(systemStateToUpdate.lightState){ // Shift traffic based on state of the light

			case GREEN: // if light is green
				tempTrafficState >>= 1; //shift whole traffic to right
			break;
			case RED: /* If Light is red or yellow */
			case YELLOW:

				beforeIntersectionTemp = tempTrafficState >> 1; //apply shift
				beforeIntersectionTemp &= MASK_BEFORE_INTERSECTION; //& with before intersection mask to isolate 8 MSB
				uint32_t trafficBuildupMask = MASK_INITIAL_TRAFFIC_BUILDUP; //use cascading mask to correct leading cars

				while(trafficBuildupMask & tempTrafficState){ //iterate down traffic until there is an empty spot
					beforeIntersectionTemp |= trafficBuildupMask; //or mask with temp to maintain leading cars
					trafficBuildupMask <<= 1; //shift mask for next space
				}

				afterIntersectionTemp = tempTrafficState & MASK_AFTER_INTERSECTION; //isolate after intersection bits
				afterIntersectionTemp >>= 1; //shift to the right
				tempTrafficState = beforeIntersectionTemp | afterIntersectionTemp; // combine pre and post traffic

			break;
		}

		tempTrafficState |= advanceCar? MASK_ADD_CAR : 0x000000000; //add car to beginning of traffic if advanceCar is true
		tempTrafficState &= MASK_19_BIT; //normalize end result to 19-bits
		systemStateToUpdate.trafficState = tempTrafficState; //write result to systemStateToUpdate

		xQueueSend(xFlowAdjustmentQueue, &currentFlow, 0); // Re-send Flow State to queue
		xQueueSend(xSystemStateQueue, &systemStateToUpdate, 0); // Send out systemState struct to queue
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/*-----------------------------------------------------------*/

/*
- Working on more accurately relating the green, red and traffic flow values
- Yellow light has been defined at 2 seconds, can be changed easily but needs to be tested as well
- Green and red values have been proportionally related
- FreeRTOS tasks have been cleanly implemented
- Dialed in red and green proportinal values

TODO:
- Implement FreeRTOS timers and delays
- Rework code to work with those timers
- Ensure task switching happens in the right time and the right way
- Test all task functions together
*/


static void prvTrafficLightStateTask ( void *pvParameters )
{
	SystemState systemStateToUpdate; //systemState to update
	FlowState currentFlow; // Current traffic flow

	uint16_t green_duration = 4000; // Duration of green light, also used for red light. Adjusted based on traffic status
	uint16_t yellow_duration = 4000; // Constant
	uint16_t red_duration; // Will be calculated based on green duration and traffic status

	//Check traffic flow rate
	if(systemStateToUpdate.trafficState == LIGHT_TRAFFIC){
		red_duration = 2*green_duration/3; // Red light is 2/3 normal green light
		green_duration /= 3; // Green light is 1/2 red light
	}else if(systemStateToUpdate.trafficState == MODERATE_TRAFFIC){
		red_duration = 3*green_duration/5; //Red light is 3/5 normal green light
		green_duration = 2*green_duration/5; // Green light close but slightly shorter than red
	}else if(systemStateToUpdate.trafficState == HIGH_TRAFFIC){
		red_duration = 2*green_duration/5; //Red light is 2/5 normal green
		green_duration = 3*green_duration/5; // Green light slightly longer than red
	}else if(systemStateToUpdate.trafficState == HEAVY_TRAFFIC){
		red_duration = green_duration/3; // Red light is 1/3 normal green light
		green_duration = 2*green_duration/3; // Green light is 2 times red light
	}

	while(1)
	{
		xQueueRecieve(xFlowAdjustmentQueue, &currentFlow, 100); // Receive current Flow State
		xQueueRecieve(xSystemStateQueue, &systemStateToUpdate, 100); // Receive current System State to update

		if( xTimerIsTimerActive( Light_Timer ) == pdFALSE )
		{
			if(systemStateToUpdate.lightState == GREEN) // If the light state is green, check this to properly implement the timer
			{

				xTimerChangePeriod(Light_Timer, pdMS_TO_TICKS(green_duration), 100);
				xTimerStart(Light_Timer, 100);

			}else if (systemStateToUpdate.lightState == YELLOW){

				xTimerChangePeriod(Light_Timer, pdMS_TO_TICKS(yellow_duration), 100);
				xTimerStart(Light_Timer, 100);

			}else if(systemStateToUpdate.lightState == RED){

				xTimerChangePeriod(Light_Timer, pdMS_TO_TICKS(red_duration), 100);
				xTimerStart(Light_Timer, 100);

			}
		}else{
			if(systemStateToUpdate.trafficState == LIGHT_TRAFFIC){
				if(systemStateToUpdate.lightState == GREEN) // If the light state is green, check this to properly implement the timer
				{
					if(xTimerGetPeriod( Light_Timer ) != green_duration) xTimerChangePeriod(Light_Timer, green_duration, 100);
				}else if (systemStateToUpdate.lightState == YELLOW){
					if(xTimerGetPeriod( Light_Timer ) != yellow_duration) xTimerChangePeriod(Light_Timer, yellow_duration, 100);
				}else if(systemStateToUpdate.lightState == RED){
					if(xTimerGetPeriod( Light_Timer ) != red_duration) xTimerChangePeriod(Light_Timer, red_duration, 100);
				}
			}else if(systemStateToUpdate.trafficState == MODERATE_TRAFFIC){
				if(systemStateToUpdate.lightState == GREEN) // If the light state is green, check this to properly implement the timer
				{
					if(xTimerGetPeriod( Light_Timer ) != green_duration) xTimerChangePeriod(Light_Timer, green_duration, 100);
				}else if (systemStateToUpdate.lightState == YELLOW){
					if(xTimerGetPeriod( Light_Timer ) != yellow_duration) xTimerChangePeriod(Light_Timer, yellow_duration, 100);
				}else if(systemStateToUpdate.lightState == RED){
					if(xTimerGetPeriod( Light_Timer ) != red_duration) xTimerChangePeriod(Light_Timer, red_duration, 100);
				}
			}else if(systemStateToUpdate.trafficState == HIGH_TRAFFIC){
				if(systemStateToUpdate.lightState == GREEN) // If the light state is green, check this to properly implement the timer
				{
					if(xTimerGetPeriod( Light_Timer ) != green_duration) xTimerChangePeriod(Light_Timer, green_duration, 100);
				}else if (systemStateToUpdate.lightState == YELLOW){
					if(xTimerGetPeriod( Light_Timer ) != yellow_duration) xTimerChangePeriod(Light_Timer, yellow_duration, 100);
				}else if(systemStateToUpdate.lightState == RED){
					if(xTimerGetPeriod( Light_Timer ) != red_duration) xTimerChangePeriod(Light_Timer, red_duration, 100);
				}red_duration = 2*green_duration/5; //Red light is 2/5 normal green
				green_duration = 3*green_duration/5; // Green light slightly longer than red
			}else if(systemStateToUpdate.trafficState == HEAVY_TRAFFIC){
				if(systemStateToUpdate.lightState == GREEN) // If the light state is green, check this to properly implement the timer
				{
					if(xTimerGetPeriod( Light_Timer ) != green_duration) xTimerChangePeriod(Light_Timer, green_duration, 100);
				}else if (systemStateToUpdate.lightState == YELLOW){
					if(xTimerGetPeriod( Light_Timer ) != yellow_duration) xTimerChangePeriod(Light_Timer, yellow_duration, 100);
				}else if(systemStateToUpdate.lightState == RED){
					if(xTimerGetPeriod( Light_Timer ) != red_duration) xTimerChangePeriod(Light_Timer, red_duration, 100);
				}
			}
		}
	}
}

/*-----------------------------------------------------------*/

/* Task Working as Callback function for Timer Dealing with Light Change */
static void vTrafficLightTimerCallback ( xTimerHandle timerHandler )
{
	SystemState systemStateToUpdate;
	xQueueRecieve (xSystemStateQueue, &systemStateToUpdate, 100); //recieve from queue

		/* change the light to the next state */
		switch (systemStateToUpdate.lightState){
			case GREEN:
				systemStateToUpdate.lightState = YELLOW;
			break;
			case YELLOW:
				systemStateToUpdate.lightState = RED;
			break;
			case RED:
				systemStateToUpdate.lightState = GREEN;
			break;
		}

	xQueueSend(xSystemStateQueue, &systemStateToUpdate, 100); //send updated light to queue
}

/*-----------------------------------------------------------*/

/* Task Working as Callback function for Timer Dealing with Light Change */
static void vTrafficLightTimerCallback ( xTimerHandle timerHandler )
{
	SystemState systemStateToUpdate;
	xQueueRecieve (xSystemStateQueue, &systemStateToUpdate, 100); //recieve from queue

		/* change the light to the next state */
		switch (systemStateToUpdate.lightState){
			case GREEN:
				systemStateToUpdate.lightState = YELLOW;
			break;
			case YELLOW:
				systemStateToUpdate.lightState = RED;
			break;
			case RED:
				systemStateToUpdate.lightState = GREEN;
			break;
		}

	xQueueSend(xSystemStateQueue, &systemStateToUpdate, 100); //send updated light to queue
}

/*-----------------------------------------------------------*/
static void prvSystemDisplayTask ( void *pvParameters )
{
	SystemState systemStateToWrite; //initialize system state to recieve values from queue

	while(1){
		xQueueReceive(xSystemStateQueue, &systemStateToWrite, portMAX_DELAY); //recieve updated System State from queue

		updateSystem(&systemStateToWrite); //update system (update lights and traffic)

		xQueueSend(xSystemStateQueue, &systemStateToWrite, 0); // send boardstate back onto queue

		vTaskDelay(pdMS_TO_TICKS(500));
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
