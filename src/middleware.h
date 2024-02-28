#ifndef MIDDLEWARE_H
#define MIDDLEWARE_H

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"

// Enum for state of traffic light
typedef enum {
    RED,
    YELLOW,
    GREEN
} LightState;

// Enum for state of traffic heaviness
typedef enum {
    LIGHT_TRAFFIC,
    MODERATE_TRAFFIC,
    HIGH_TRAFFIC,
    HEAVY_TRAFFIC
} FlowState;

// struct to hold state of whole system
typedef struct {
    LightState lightState;
    uint32_t trafficState;
} SystemState;

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

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void ADCInit( void );
void TIM2Init( void );
void GPIOInit( void );
void MiddleWareSetDefault ( void );
uint16_t readPot ( void );
void updateTrafficLight ( SystemState* state );
void updateTraffic ( SystemState* state );
void updateSystem (SystemState* state);

#endif