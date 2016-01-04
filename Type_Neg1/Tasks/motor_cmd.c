#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "motor_cmd.h"
#include "PID.h"
#include "ipc_struct.h"


/* This is Motor Commander driver. This commander only take
velocity and steering radius. Also this commander takes care
of Speed Feedback. Speed Feedback should be synchronised with 
motor control loop. */

// ATTENTION: PID parameters may need be flexible, because the
// desire PID parameters will change at different speed.
// Should note this problem in post

// Macros
// Loop duration in ms
#define MOTORCMD_LOOP_DURATION (100)
// Distance between steering wheels to sensor in mm
#define STEERING_DISTANCE (200)
// MotorCMD Task SIZE
#define MOTORCMD_SIZE (330)
// Wheel diameter in mm
#define WHEEL_DIAMETER (40)
// Distance between wheels in mm
#define WHEEL_DISTANCE (120)
// PI
#define MOTOR_PI (3.1415)
// Left FB TIM
#define LEFT_FB TIM3
// Right FB TIM
#define RIGHT_FB TIM4
// Left motor
#define LEFT_MOTOR TIM9
// Right Motor
#define RIGHT_MOTOR TIM12
// Enable Feedback
//#define MOTOR_FEEDBACK_ENABLE
// Enable PID
//#define MOTOR_PID_ENABLE
// Enable Queue
//#define MOTOR_QUEUE_ENABLE
// Encoder mode
 #define TIM_ENCODER_ENABLE
//Maxspeed
#define MAX_SPEED 10.0
// DEBUG MODE Enable
#define MOTOR_DEBUG


// Extern variables
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern xQueueHandle msg_speed;
extern xQueueHandle msg_motor_DEBUG;



// Function prototypes
static void vMotorCMDTask( void *pvParameters );
static double _wheel_speed_calc(int counter);
static void _motor_FB(double* left, double* right);
static void _clear_FB(void);
static uint16_t _speed_to_pwm(double speed);
static void _get_set_speed(Vehicle_vector_t msg_speed, double* Left_Speed_set, double* Right_Speed_set);



// Private variables
PID_Profile Left_Motor_PID;
PID_Profile Right_Motor_PID;
uint16_t motor_PWM_DEBUG = 0;
/**
  * @brief  start the motor commander task, initializing all the TIMs and counters
  * @param  task priority. 
  * @retval None
  */

void vStartMotorCMDTask( UBaseType_t uxPriority )
{

    // Start TIMs and Counters
		HAL_TIM_Base_Start(&htim9);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    
    HAL_TIM_Base_Start(&htim12);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);

    #ifdef TIM_ENCODER_ENABLE
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    #endif

    // Initialize the motors
    PID_setup(&Left_Motor_PID, 0, 0, 0, 0, 0);
    PID_setup(&Right_Motor_PID, 0, 0, 0, 0, 0);
	/* Spawn the task. */
    xTaskCreate( vMotorCMDTask, "Motor_CMD", MOTORCMD_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	//}
}

/**
  * @brief actual motor control thread. This task checks velocity and steering radius from IPC
  then calculate the expected speed on both wheels, and then put all the things in PID core.
  Also this task will be excuted with duration defiend by MOTORCMD_LOOP_DURATION in ms
  * @param  will not be used 
  * @retval None
  */
static void vMotorCMDTask( void *pvParameters )
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = MOTORCMD_LOOP_DURATION/portTICK_PERIOD_MS;
    

    double Left_Speed_now=0;
    double Right_Speed_now=0;
    double Left_Speed_set = 0;
    double Right_Speed_set = 0;
    double Left_Speed_output = 0;
    double Right_Speed_output = 0;
    uint16_t Left_PWM = 0;
    uint16_t Right_PWM = 0;
    Vehicle_vector_t car_speed;
    portBASE_TYPE status;
	
    for(;;)
	{
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        #ifdef MOTOR_FEEDBACK_ENABLE
		// Read FB
        _motor_FB(&Left_Speed_now, &Right_Speed_now);
        // Clear the FB
        _clear_FB();
        #endif
        
        #ifdef MOTOR_QUEUE_ENABLE
        // Check Queue
        status = xQueueReceive(msg_speed, &car_speed, 0); //No delay
        if (status == pdFALSE){
          // no update yet
          continue;
        }else{
          _get_set_speed(car_speed, &Left_Speed_set, &Right_Speed_set);
        }
        #endif
        
        #ifdef MOTOR_PID_ENABLE
        // Run PID and put output to the TIM
        Left_Speed_output = PID_run(&Left_Motor_PID, (Left_Speed_set-Left_Speed_now), Left_Speed_now);
        Right_Speed_output = PID_run(&Right_Motor_PID, (Right_Speed_set-Right_Speed_now), Right_Speed_now);
        #endif
    
        Left_PWM = _speed_to_pwm(Left_Speed_output);
        Right_PWM = _speed_to_pwm(Right_Speed_output);
				
				#ifdef MOTOR_DEBUG
				// in this mode, user use btn to increase the motor PWM by 100.
				// Maximum pwm is 1000
				uint8_t btn_flag = 0;
				status = xQueueReceive(msg_motor_DEBUG, &btn_flag, 0); //No delay
				if (status == pdFALSE){
					// no update yet
					continue;
				}else{
					if (motor_PWM_DEBUG <= 900){
						motor_PWM_DEBUG += 100;
					}else{
						motor_PWM_DEBUG = 0;
					}
				}
				Left_PWM = motor_PWM_DEBUG;
				Right_PWM = motor_PWM_DEBUG;
				#endif
        
        // ATTENTION: THIS VERSION DOES NOT HAVE BRAKE!
        LEFT_MOTOR->CCR1 = Left_PWM;
        RIGHT_MOTOR->CCR1 = Right_PWM;
        
	}
}



/**
  * @brief Read acutal speed of left motor and right motor to m/s
  * @param  pointer to left_speed
  * @param  pointer to right_speed
  * @retval None
  */
static void _motor_FB(double* left, double* right){
    double temp=0;
    int counter_temp = 0;

    // read left counter
    counter_temp = LEFT_FB->CNT;
    temp = _wheel_speed_calc(counter_temp);
    *left = (double)temp;

    // read right counter
    counter_temp = LEFT_FB->CNT;
    temp = _wheel_speed_calc(counter_temp);
    *right = (double)temp;

    return;
}

/**
  * @brief clear the Left and Right feedback counter
  * @param  None
  * @retval None
  */
static void _clear_FB(void){
    portENTER_CRITICAL();
    LEFT_FB->CNT = 0;
    RIGHT_FB->CNT = 0;
    portEXIT_CRITICAL();
    return;
}

/**
  * @brief calculate wheel speed from feedback
  * @param  int counter
  * @retval double speed
  */
static double _wheel_speed_calc(int counter){
  return (double)(2.0*MOTOR_PI*WHEEL_DIAMETER)*(counter)*(1000/MOTORCMD_LOOP_DURATION)*60;
}

/**
  * @brief transform actual speed to PWM
  * @param  actual speed in m/s
  * @retval uint16_t PWM
  */
static uint16_t _speed_to_pwm(double speed){
    if (speed > MAX_SPEED){
        speed = MAX_SPEED;
    }
    return (uint16_t)floor(1000.0*speed/MAX_SPEED);
}

/**
  * @brief caculate the desired speed for each wheel
  * @param  desired speed
  * @param  pointer to the desired left wheel speed
  * @param  pointer to the desired right wheel speed
  * @retval uint16_t PWM
  */
static void _get_set_speed(Vehicle_vector_t msg_speed, double* Left_Speed_set, double* Right_Speed_set){
  double radius = msg_speed.radius;
  double raw_speed = msg_speed.speed;

  *Left_Speed_set = (double)((radius+(WHEEL_DISTANCE/1000.0))/(radius))*(raw_speed);
  *Right_Speed_set = (double)((radius-(WHEEL_DISTANCE/1000.0))/(radius))*(raw_speed);

  return;
}
