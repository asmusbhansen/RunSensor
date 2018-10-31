#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

/* For enabling TWI set the same TWI settings as in the sdk_config of the twi_sensor example and add both nrfx_twi.c and nrf_drv_twi.c to the project */ 

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
//#define MPU9250_ADDR          (0x68U >> 1)
#define MPU_ADDRESS      0x68

#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
#define GYRO_CONFIG      0x1B 
#define ACCEL_CONFIG     0x1C

#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48


#define MPU_TWI_BUFFER_SIZE       14


/* MPU TWI Output pins */
#define MPU_TWI_SCL_PIN ARDUINO_SCL_PIN
#define MPU_TWI_SDA_PIN ARDUINO_SDA_PIN

/* Timeout for MPU9250 */
#define MPU_TWI_TIMEOUT 	  10000 


/* Error codes */
#define MPU9250_ERROR_CODE_OFFSET 0x00
#define MPU9250_NO_ERROR          0x00 + MPU9250_ERROR_CODE_OFFSET
#define MPU9250_NO_CONNECTION     0x01 + MPU9250_ERROR_CODE_OFFSET



/* Mode for MPU9250. */
#define NORMAL_MODE 0U


/* Struct for storing MPU9250 sensor values */
typedef struct 
{
    int16_t accl_X;                                  
    int16_t accl_Y;
    int16_t accl_Z;
    int16_t temp;
    int16_t gyro_X;                                  
    int16_t gyro_Y;
    int16_t gyro_Z;

}mpu9250_sensor_values;

/**@brief Function for waking up the MPU9250.
 *
 * @details This function verifies connection to the device and clears the PWR_MGMT_1 to exit sleep mode
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     MPU9250_NO_ERROR on success, otherwise an error code.
 */

uint32_t mpu9250_wake(void);


/**@brief Function for initializing the TWI connection to the MPU9250.
 *
 * @details This function initializes TWI connection
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     MPU9250_NO_ERROR on success, otherwise an error code.
 */
uint32_t nrf_drv_mpu_init(void);

/**@brief Function for reading MPU9250 sensors.
 *
 * @details This function reads MPU9250 sensors and returns the values as 16 bit signed integers
 *
 * @note 
 *       
 * @param[in]  sensor_values - Struct for storing MPU9250 sensor values
 *
 * @return     none
 */
void read_mpu_sensors(mpu9250_sensor_values *sensor_values);