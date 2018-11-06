#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"

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

#define GYRO_SCALE       131

#define UPDATE_LOOP_DT   1

/* MPU TWI Output pins */
#define MPU_TWI_SCL_PIN ARDUINO_SCL_PIN
#define MPU_TWI_SDA_PIN ARDUINO_SDA_PIN

/* Timeout for MPU9250 */
#define MPU_TWI_TIMEOUT 	  10000 


/* Error codes */
#define MPU9250_ERROR_CODE_OFFSET 0x00
#define MPU9250_NO_ERROR          0x00 + MPU9250_ERROR_CODE_OFFSET
#define MPU9250_NO_CONNECTION     0x01 + MPU9250_ERROR_CODE_OFFSET

#define PI_M             3.1415

/* Mode for MPU9250. */
#define NORMAL_MODE 0U

/* RAM Space for TWI DMA transfers*/ 
#define TWIM_RX_BUF_LENGTH  100
#define TWIM_RX_BUF_WIDTH   MPU_TWI_BUFFER_SIZE


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

/* Struct for storing MPU9250 orientation */
typedef struct 
{
    double mpu_xz_angle;                                  
    double mpu_yz_angle;

}MPU9250_orientation;

/* Define a type with a two dimensioanal array, TWIM_RX_BUF_WIDTH wide and TWIM_RX_BUF_LENGTH long, holding a list of MPU sensor data */
typedef struct ArrayList
{
    uint8_t buffer[TWIM_RX_BUF_WIDTH];
}array_list_t;



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


/**
 * @brief TWI deactivation
 */
/**@brief Function to deactivate the TWI connection to the MPU9250.
 *
 * @details This function deactivates TWI connection
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     MPU9250_NO_ERROR on success, otherwise an error code.
 */
void nrf_drv_mpu_deavtivate(void);

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


 /**@brief Function for initializing TWI DMA transfer
 *
 * @details This function starts the TWI DMA transfer continously
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     none
 */
void mpu_twi_dma_init();

 /**@brief Function for printing TWI DMA buffer
 *
 * @details This function prints the DMA buffer
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     none
 */
void print_dma_buffer();


 /**@brief Function for starting TWI DMA transfer
 *
 * @details This function starts the DMA buffer
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     none
 */
void start_twi_dma_transfer();

 /**@brief Function for stopping TWI DMA transfer
 *
 * @details This function stops the DMA buffer
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     none
 */
void stop_twi_dma_transfer();


 /**@brief Function for processing MPU data
 *
 * @details This function procceses the MPU data readings
 *
 * @note 
 *       
 * @param[in]  none
 *
 * @return     none
 */
void process_mpu_data();

 /**@brief Function for reading MPU data
 *
 * @details This function reading the MPU data
 *
 * @note 
 *       
 * @param[in]  Struct for the MPU values and a relative location of the data in RAM
 *
 * @return     none
 */
void read_mpu_data_RAM(mpu9250_sensor_values *sensor_values, uint32_t location);


/**@brief Function for for obtaining MPU orientation struct
 *
 * @details This function returns a copy of the orientation struct
 *
 * @note 
 *       
 * @param[in]  Struct for the MPU orientation
 *
 * @return     none
 */
void get_mpu_orientation(MPU9250_orientation * orientation);