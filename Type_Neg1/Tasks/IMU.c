#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "IMU.h"
#include "ipc_struct.h"

// Loop duration in ms
#define IMU_LOOP_DURATION (100)

// Select IMU chip
#define BNO_055_SEL
// Select UART port
extern UART_HandleTypeDef huart2;
#define IMU_HUART huart2
// Define IMU_SIZE
#define IMU_SIZE (200)
// Define the I2C handle
extern I2C_HandleTypeDef *hi2c1;



static void vIMUTask( void *pvParameters );

void vStartIMUTask( UBaseType_t uxPriority )
{

    // Start IMU
	#ifdef BNO_055_SEL
    HAL_UART_Transmit(&huart2, (uint8_t*)"[IMU]: init\n", 20, 1/portTICK_PERIOD_MS);
    HAL_I2C_Mem_Write(hi2c1, 0x28, 0x11, 1, (uint8_t*)0x11, 1, 100/portTICK_PERIOD_MS);
    
    if(HAL_I2C_IsDeviceReady(hi2c1, 0x28<<1 , 5, 100/portTICK_PERIOD_MS)){
        HAL_UART_Transmit(&huart2, (uint8_t*)"[IMU]: Device Ready\n", 20, 1/portTICK_PERIOD_MS);
        BNO_055_begin(OPERATION_MODE_IMUPLUS, hi2c1, 0x28);
        BNO_055_setExtCrystalUse(true);
        }else{
            HAL_UART_Transmit(&huart2, (uint8_t*)"[IMU]: Device Not Ready\n", 20, 1/portTICK_PERIOD_MS);
        }
    #else
	#endif


	/* Spawn the task. */
    xTaskCreate( vIMUTask, "IMU", IMU_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	//}
}



static void vIMUTask( void *pvParameters )
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = IMU_LOOP_DURATION/portTICK_PERIOD_MS;
 
	
    for(;;)
	{
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
