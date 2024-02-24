#ifndef UTILITIES_H
#define UTILITIES_H

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
/* Defined Includes*/
#include "middleware.h"

//defines for potentiometer
#define POT_MIN 0
#define POT_MAX 3500

//bool used for traffic generation
typedef int bool;
#define true 1
#define false 0

//Masks used for traffic generation
#define MASK_19_BIT 0xFFFFE000U
#define MASK_BEFORE_INTERSECTION 0xFF000000U
#define MASK_AFTER_INTERSECTION 0xFFE000U

//defines for system clock
#define LONG_LIGHT_DELAY 1000
#define LIGHT_TRAFFIC_DELAY (LONG_LIGHT_DELAY/2) 
#define MODERATE_TRAFFIC_DELAY (LONG_LIGHT_DELAY/2)
#define HIGH_TRAFFIC_DELAY
#define HEAVY_TRAFFIC_DELAY

void updateTrafficGreen ( SystemState* state );
void updateTrafficRedOrYellow (SystemState* state);

#endif