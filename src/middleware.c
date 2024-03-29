#include "middleware.h"

void ADCInit (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enables the ADC1 Clock
	ADC_InitTypeDef ADCInit_Structure; // Initialize ADC Init Struct
	ADC_StructInit(&ADCInit_Structure); // Provide Default Configurations
	ADC_Init(ADC1, &ADCInit_Structure); // Enables and configures the prescaler, unsure if these are the correct values
	ADC_Cmd(ADC1, ENABLE); // Activates the ADC peripheral
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_144Cycles); // Sets it to channel 0
}

void GPIOInit (void)
{
    // Enable Clock for GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

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

uint16_t readPot (void)
{
    uint16_t ADC_Value;
    ADC_SoftwareStartConv(ADC1); // Start ADC conversions
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET){} // Check for end of conversion flag
	ADC_Value = ADC_GetConversionValue(ADC1); // Read ADC value after conversion
    return ADC_Value; // return value read by ADC 
}

void MiddleWareSetDefault ( void )
{
    
}

void updateTrafficLight ( SystemState* state )
{
    switch (state->lightState)
    {
        case RED:
            GPIOC->ODR = (GPIOC->ODR & ~(LIGHT_YELLOW | LIGHT_GREEN)) | LIGHT_RED;
        break;
        case YELLOW:
            GPIOC->ODR = (GPIOC->ODR & ~(LIGHT_RED | LIGHT_GREEN)) | LIGHT_YELLOW;
        break;
        case GREEN:
            GPIOC->ODR = (GPIOC->ODR & ~(LIGHT_RED | LIGHT_YELLOW)) | LIGHT_GREEN;
        break;
    }   
}

void updateTraffic ( SystemState* state )
{
    uint32_t trafficShiftMask = TRAFFIC_START_MASK;
    for(int i = 0; i < 19; i++)
    {
        // if traffic state masked with current traffic bit is nonzero 
        if (state->trafficState & trafficShiftMask) GPIOC->ODR |= SR_DATA;
        else GPIOC->ODR &= ~SR_DATA;
        
        GPIO_ToggleBits(GPIOC, SR_CLK); GPIO_ToggleBits(GPIOC, SR_CLK);  // Toggle Shift Register Clock up and down 

        trafficShiftMask >>= 1; // shift traffic shift mask to right to 
    }
    GPIOC->ODR &= ~SR_CLK | ~SR_DATA; // set SR clock and data to low
} 

updateSystem (SystemState* state)
{
	updateTrafficLight(state);
	updateTraffic(state);
}