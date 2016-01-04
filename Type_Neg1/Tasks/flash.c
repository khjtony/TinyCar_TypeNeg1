#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
//#include "partest.h"
#include "flash.h"


#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 1 )
#define ledFLASH_RATE_BASE	( ( TickType_t ) 333 )

/* Variable used by the created tasks to calculate the LED number to use, and
the rate at which they should flash the LED. */
static volatile UBaseType_t uxFlashTaskNumber = 0;

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLEDFlashTask, pvParameters );

/*-----------------------------------------------------------*/

void vStartLEDFlashTasks( UBaseType_t uxPriority )
{
BaseType_t xLEDTask;

	/* Create the three tasks. */
	//for( xLEDTask = 0; xLEDTask < ledNUMBER_OF_LEDS; ++xLEDTask )
	//{
		/* Spawn the task. */
    xTaskCreate( vLEDFlashTask, "LEDx", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	//}
}
/*-----------------------------------------------------------*/


static void vLEDFlashTask( void *pvParameters )
{

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		//vTaskDelayUntil( &xLastFlashTime, xFlashRate );
		vTaskDelay(500/portTICK_RATE_MS);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		/* Delay for half the flash period then turn the LED off. */
		//vTaskDelayUntil( &xLastFlashTime, xFlashRate );
		vTaskDelay(500/portTICK_RATE_MS);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	}
}

