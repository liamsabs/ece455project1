#ifndef MIDDLEWARE_H
#define MIDDLEWARE_H

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"

// Enum for state of traffic light
typedef enum 
{
    RED,
    YELLOW,
    GREEN
} LightState;

// struct to hold state of whole system
struct SystemState
{
    LightState lightState;
    uint32_t trafficState;
};

// Defines for Traffic Lights
#define LIGHT_RED GPIO_ODR_ODR_0
#define LIGHT_YELLOW GPIO_ODR_ODR_1
#define LIGHT_GREEN GPIO_ODR_ODR_2

// Defines for Shift Register
#define SR_DATA GPIO_ODR_ODR_6 //data in for serial register system
#define SR_CLK GPIO_ODR_ODR_8 //clock for serial data (loaded on rising edge)
#define SR_CLR GPIO_ODR_ODR_9 //clears sr by writing zero to it

//Defines for traffic management
#define TRAFFIC_START_MASK 0x80000000U //used for loading to board

void ADCInit( void );
void GPIOInit( void );
uint16_t readPot ( void );
void updateTrafficLight ( struct SystemState* state );
void updateTraffic ( struct SystemState* state );

#endif
