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
typedef uint8_t bool;
#define true 1
#define false 0

//Masks used for traffic generation
#define MASK_19_BIT (uint32_t)0xFFFFE000U
#define MASK_BEFORE_INTERSECTION (uint32_t)0xFF000000U
#define MASK_AFTER_INTERSECTION (uint32_t)0xFFE000U
#define MASK_INITIAL_TRAFFIC_BUILDUP (uint32_t)0x1000000U
#define MASK_ADD_CAR (uint32_t)0x80000000U

//defines for system clock
#define LONG_LIGHT_DELAY 1000
#define LIGHT_TRAFFIC_DELAY (LONG_LIGHT_DELAY/2) 
#define MODERATE_TRAFFIC_DELAY (LONG_LIGHT_DELAY/2)
#define HIGH_TRAFFIC_DELAY
#define HEAVY_TRAFFIC_DELAY


#endif