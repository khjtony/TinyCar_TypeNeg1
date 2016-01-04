/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/


#include <math.h>
#include <limits.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */

#include "BNO055.h"
// Select UART port
extern UART_HandleTypeDef huart2;


/***************************************************************************
 Prototype and static functions
 ***************************************************************************/



static uint8_t  _read8   ( adafruit_bno055_reg_t );
static bool  _readLen ( adafruit_bno055_reg_t, uint8_t* buffer, uint8_t len );
static bool  _write8  ( adafruit_bno055_reg_t, uint8_t value );
static adafruit_bno055_opmode_t _mode;
static uint8_t _address;
static int32_t _sensorID;
static I2C_HandleTypeDef* _hi2c;

/* Functions to deal with raw calibration data */
// bool  BNO_055_getSensorOffsets(uint8_t* calibData);

// void  BNO_055_setSensorOffsets(const uint8_t* calibData);




/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
bool BNO_055_begin(adafruit_bno055_opmode_t mode, I2C_HandleTypeDef *in_hi2c, uint8_t device_addr)
{
    _hi2c = in_hi2c;
    _address = device_addr;
    
  /* Make sure we have the right device */

    uint8_t id = _read8(BNO055_CHIP_ID_ADDR);
 
  if(id != BNO055_ID)
  {
    id = _read8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID) {
      return false;  // still not? ok bail
    }
  }
 
    
  /* Switch to config mode (just in case since this is the default) */
  BNO_055_setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  _write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (_read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
  vTaskDelay(50/portTICK_PERIOD_MS);

  /* Set to normal power mode */
  _write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  vTaskDelay(10/portTICK_PERIOD_MS);

  _write8(BNO055_PAGE_ID_ADDR, 0);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  _write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  _write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  _write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */
  
  _write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  vTaskDelay(10/portTICK_PERIOD_MS);
  /* Set the requested operating mode (see section 3.3) */
  BNO_055_setMode(mode);
  vTaskDelay(20/portTICK_PERIOD_MS);

  return true;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
void BNO_055_setMode(adafruit_bno055_opmode_t mode)
{
  _mode = mode;
  _write8(BNO055_OPR_MODE_ADDR, _mode);
  vTaskDelay(30/portTICK_PERIOD_MS);
}

/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void BNO_055_setExtCrystalUse(bool usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  BNO_055_setMode(OPERATION_MODE_CONFIG);
  vTaskDelay(25/portTICK_PERIOD_MS);
  _write8(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    _write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    _write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  vTaskDelay(10/portTICK_PERIOD_MS);
  /* Set the requested operating mode (see section 3.3) */
  BNO_055_setMode(modeback);
  vTaskDelay(20/portTICK_PERIOD_MS);
}


/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void BNO_055_getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  _write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = _read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = _read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error     = _read8(BNO055_SYS_ERR_ADDR);

  vTaskDelay(200/portTICK_PERIOD_MS);
}

/**************************************************************************/
/*!
    @brief  Gets the chip revision numbers
*/
/**************************************************************************/
void BNO_055_getRevInfo(adafruit_bno055_rev_info_t* info)
{
  //In order to use this part, remember to initialize the info
  uint8_t a, b;


  /* Check the accelerometer revision */
  info->accel_rev = _read8(BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev   = _read8(BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev  = _read8(BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev    = _read8(BNO055_BL_REV_ID_ADDR);

  a = _read8(BNO055_SW_REV_ID_LSB_ADDR);
  b = _read8(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
void BNO_055_getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  uint8_t calData = _read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
int8_t BNO_055_getTemp(void)
{
  int8_t temp = (int8_t)(_read8(BNO055_TEMP_ADDR));
  return temp;
}

/**************************************************************************/
/*!
    @brief  Gets a vector reading from the specified source
*/
/**************************************************************************/
BNO_055_vector_t BNO_055_getVector(adafruit_vector_type_t vector_type)
{
  BNO_055_vector_t xyz;
  uint8_t buffer[6];

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 uint8_ts) */
  _readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /* Convert the value to an appropriate range (section 3.6.4) */
  /* and assign the value to the Vector type */
  switch(vector_type)
  {
    case VECTOR_MAGNETOMETER:
      /* 1uT = 16 LSB */
      xyz.x = ((double)x)/16.0;
      xyz.y = ((double)y)/16.0;
      xyz.z = ((double)z)/16.0;
      break;
    case VECTOR_GYROSCOPE:
      /* 1rps = 900 LSB */
      xyz.x = ((double)x)/900.0;
      xyz.y = ((double)y)/900.0;
      xyz.z = ((double)z)/900.0;
      break;
    case VECTOR_EULER:
      /* 1 degree = 16 LSB */
      xyz.x = ((double)x)/16.0;
      xyz.y = ((double)y)/16.0;
      xyz.z = ((double)z)/16.0;
      break;
    case VECTOR_ACCELEROMETER:
    case VECTOR_LINEARACCEL:
    case VECTOR_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      xyz.x = ((double)x)/100.0;
      xyz.y = ((double)y)/100.0;
      xyz.z = ((double)z)/100.0;
      break;
  }

  return xyz;

}

/**************************************************************************/
/*!
    @brief  Gets a quaternion reading from the specified source
*/
/**************************************************************************/
BNO_055_Quat_t BNO_055_getQuat(void)
{
    BNO_055_Quat_t quat;
    uint8_t buffer[8];

    int16_t x, y, z, w;
    x = y = z = w = 0;

    /* Read quat data (8 uint8_ts) */
    _readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    /* Assign to Quaternion */
    /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
    const double scale = (1.0 / (1<<14));
    quat.x = scale * x;
    quat.y = scale * y;
    quat.z = scale * z;
    quat.w = scale * w;

    return quat;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void BNO_055_getSensor(UART_HandleTypeDef *in_huart)
{
  // /* Clear the sensor_t object */
  // memset(sensor, 0, sizeof(sensor_t));

  // /* Insert the sensor name in the fixed length char array */
  // strncpy (sensor->name, "BNO055", sizeof(sensor->name) - 1);
  // sensor->name[sizeof(sensor->name)- 1] = 0;
  // sensor->version     = 1;
  // sensor->sensor_id   = _sensorID;
  // sensor->type        = SENSOR_TYPE_ORIENTATION;
  // sensor->min_delay   = 0;
  // sensor->max_value   = 0.0F;
  // sensor->min_value   = 0.0F;
  // sensor->resolution  = 0.01F;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
void BNO_055_readSensor(UART_HandleTypeDef *in_huart)

{
  // /* Clear the event */
  // memset(event, 0, sizeof(sensors_event_t));

  // event->version   = sizeof(sensors_event_t);
  // event->sensor_id = _sensorID;
  // event->type      = SENSOR_TYPE_ORIENTATION;
  // event->timestamp = millis();

  // /* Get a Euler angle sample for orientation */
  // imu::Vector<3> euler = getVector(VECTOR_EULER);
  // event->orientation.x = euler.x();
  // event->orientation.y = euler.y();
  // event->orientation.z = euler.z();

  return;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into a uint8_t array
*/
/**************************************************************************/
// bool BNO_055_getSensorOffsets(uint8_t* calibData)
// {
//     if (isFullyCalibrated())
//     {
//         adafruit_bno055_opmode_t lastMode = _mode;
//         setMode(OPERATION_MODE_CONFIG);

//         readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

//         setMode(lastMode);
//         return true;
//     }
//     return false;
// }

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into an offset struct
*/
/**************************************************************************/
bool BNO_055_getSensorOffsets(adafruit_bno055_offsets_t* offsets_type)
{
    if (BNO_055_isFullyCalibrated())
    {
        adafruit_bno055_opmode_t lastMode = _mode;
        BNO_055_setMode(OPERATION_MODE_CONFIG);
        vTaskDelay(25/portTICK_PERIOD_MS);

        offsets_type->accel_offset_x = (_read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (_read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type->accel_offset_y = (_read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (_read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type->accel_offset_z = (_read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (_read8(ACCEL_OFFSET_Z_LSB_ADDR));

        offsets_type->gyro_offset_x = (_read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (_read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type->gyro_offset_y = (_read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (_read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type->gyro_offset_z = (_read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (_read8(GYRO_OFFSET_Z_LSB_ADDR));

        offsets_type->mag_offset_x = (_read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (_read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type->mag_offset_y = (_read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (_read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type->mag_offset_z = (_read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (_read8(MAG_OFFSET_Z_LSB_ADDR));

        offsets_type->accel_radius = (_read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (_read8(ACCEL_RADIUS_LSB_ADDR));
        offsets_type->mag_radius = (_read8(MAG_RADIUS_MSB_ADDR) << 8) | (_read8(MAG_RADIUS_LSB_ADDR));

        BNO_055_setMode(lastMode);
        return true;
    }
    return false;
}


/**************************************************************************/
/*!
@brief  Writes an array of calibration values to the sensor's offset registers
*/
// /**************************************************************************/
// void BNO_055_setSensorOffsets(const uint8_t* calibData)
// {
//     adafruit_bno055_opmode_t lastMode = _mode;
//     setMode(OPERATION_MODE_CONFIG);
//     delay(25);

//     /* A writeLen() would make this much cleaner */
//     _write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
//     _write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
//     _write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
//     _write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
//     _write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
//     _write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

//     _write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
//     _write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
//     _write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
//     _write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
//     _write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
//     _write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

//     _write8(MAG_OFFSET_X_LSB_ADDR, calibData[12]);
//     _write8(MAG_OFFSET_X_MSB_ADDR, calibData[13]);
//     _write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
//     _write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
//     _write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
//     _write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

//     _write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
//     _write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

//     _write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
//     _write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

//     setMode(lastMode);
// }

/**************************************************************************/
/*!
@brief  Writes to the sensor's offset registers from an offset struct
*/
/**************************************************************************/
void BNO_055_setSensorOffsets(const adafruit_bno055_offsets_t* offsets_type)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    BNO_055_setMode(OPERATION_MODE_CONFIG);
    vTaskDelay(25/portTICK_PERIOD_MS);

    _write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type->accel_offset_x) & 0x0FF);
    _write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type->accel_offset_x >> 8) & 0x0FF);
    _write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type->accel_offset_y) & 0x0FF);
    _write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type->accel_offset_y >> 8) & 0x0FF);
    _write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type->accel_offset_z) & 0x0FF);
    _write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type->accel_offset_z >> 8) & 0x0FF);

    _write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type->gyro_offset_x) & 0x0FF);
    _write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type->gyro_offset_x >> 8) & 0x0FF);
    _write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type->gyro_offset_y) & 0x0FF);
    _write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type->gyro_offset_y >> 8) & 0x0FF);
    _write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type->gyro_offset_z) & 0x0FF);
    _write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type->gyro_offset_z >> 8) & 0x0FF);

    _write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type->mag_offset_x) & 0x0FF);
    _write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type->mag_offset_x >> 8) & 0x0FF);
    _write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type->mag_offset_y) & 0x0FF);
    _write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type->mag_offset_y >> 8) & 0x0FF);
    _write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type->mag_offset_z) & 0x0FF);
    _write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type->mag_offset_z >> 8) & 0x0FF);

    _write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type->accel_radius) & 0x0FF);
    _write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type->accel_radius >> 8) & 0x0FF);

    _write8(MAG_RADIUS_LSB_ADDR, (offsets_type->mag_radius) & 0x0FF);
    _write8(MAG_RADIUS_MSB_ADDR, (offsets_type->mag_radius >> 8) & 0x0FF);

    BNO_055_setMode(lastMode);
}

bool BNO_055_isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    BNO_055_getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return false;
    return true;
}


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
bool _write8(adafruit_bno055_reg_t reg, uint8_t value)
{
    uint8_t pData[1];
    pData[0] = value;
    while(HAL_I2C_Mem_Write(_hi2c, _address<<1, reg, 1, pData, 1, 100/portTICK_PERIOD_MS)){
        if (HAL_I2C_GetError(_hi2c) != HAL_I2C_ERROR_AF)
       {
           HAL_UART_Transmit(&huart2, (uint8_t*)"[_write8]: Error\n", 20, 1/portTICK_PERIOD_MS);
       }
    }

    /* ToDo: Check for error! */
    return true;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t _read8(adafruit_bno055_reg_t reg )
{
    HAL_UART_Transmit(&huart2, (uint8_t*)"[BNO055]: _read8\n", 20, 1/portTICK_PERIOD_MS);
    uint8_t pData[1];
    
    /* -> Lets ask for register's address */
     while(HAL_I2C_Master_Transmit(_hi2c, (uint16_t)_address<<1, (uint8_t*)&reg, 1, (uint32_t)1000)!= HAL_OK)
    {
        /*
         * Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */
  
    }
 
    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
      while (HAL_I2C_GetState(_hi2c) != HAL_I2C_STATE_READY)
      {
      }
 
    /* -> Put I2C peripheral in reception process */
    while(HAL_I2C_Master_Receive(_hi2c, (uint16_t)_address<<1, pData, 1, (uint32_t)1000) != HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */
        if (HAL_I2C_GetError(_hi2c) != HAL_I2C_ERROR_AF)
        {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[_read8]: Error\n", 20, 1/portTICK_PERIOD_MS);
        }
    }

    while (HAL_I2C_GetState(_hi2c) != HAL_I2C_STATE_READY)
    {
    }
    
    /*
    while(HAL_I2C_Mem_Read(_hi2c, (uint16_t)_address<<1, reg, 1, pData, 1, 100/portTICK_PERIOD_MS)){
        if (HAL_I2C_GetError(_hi2c) != HAL_I2C_ERROR_AF)
       {
           HAL_UART_Transmit(&huart2, (uint8_t*)"[_read8]: Error\n", 20, 1/portTICK_PERIOD_MS);
       }
    }
*/
    /* ToDo: Check for error! */
    return pData[0];

}

/**************************************************************************/
/*!
    @brief  Reads the specified number of uint8_ts over I2C
*/
/**************************************************************************/
bool _readLen(adafruit_bno055_reg_t reg, uint8_t * buffer, uint8_t len)
{
    HAL_I2C_Mem_Read(_hi2c, (uint16_t)_address<<1, reg, 1, buffer, len, 100/portTICK_PERIOD_MS);    

  /* ToDo: Check for errors! */
  return true;
}
