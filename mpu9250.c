#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "mpu9250.h"

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TX Buffer*/
uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

/* TWI instance. */
static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            /*if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }*/
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
uint32_t nrf_drv_mpu_init(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t twi_mpu_config = {
       .scl                = MPU_TWI_SCL_PIN,
       .sda                = MPU_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
       .clear_bus_init     = false
    };
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_mpu_config, twi_handler, NULL);
    if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
    
    nrf_drv_twi_enable(&m_twi_instance);

    NRF_LOG_INFO("TWI Init"); 

    return NRF_SUCCESS;
}

/**
 * @brief Function for reading from MPU9250 register
 */
static int32_t nrf_drv_mpu_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;


    //Write the address which we want to read from, to the MPU9250
    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    // Wait for transfer finish or timeout 
    while((!m_xfer_done) && --timeout);
    if(!timeout){
      return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;

    //Read lenght x 8bit data from the MPU9250
    err_code = nrf_drv_twi_rx(&m_twi_instance, MPU_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU_TWI_TIMEOUT;
    while((!m_xfer_done) && --timeout);
    if(!timeout){
     return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;

    return err_code;
}

/**
 * @brief Function for writing MPU9250 register
 */
static uint32_t nrf_drv_mpu_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    //To communicate with the MPU9250 it is nescassary to modify the write register function, for writing 8bit values
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    //merge_register_and_data(twi_tx_buffer, reg, p_data, length);

    //Set the first byte in tx buffer to the register address
    twi_tx_buffer[0] = reg;
    //Copy length bytes from p_data to address twi_tx_buffer + 1
    memcpy((twi_tx_buffer + 1), p_data, length);

    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = MPU_ADDRESS;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

    // Transferring
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &xfer_desc, 0);

    while((!m_xfer_done) && --timeout);
    if(!timeout) {
      return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;

    return err_code;
}

/**
 * @brief Function for waking up MPU9250.
 */
uint32_t mpu9250_wake(void)
{
    ret_code_t err_code;
    uint32_t data_len = 0;
    uint8_t data[1] = {0};

    //Read 1 byte from WHO_AM_I_MPU9250 register to verify connection
    data_len = 1;
    err_code = nrf_drv_mpu_read_registers(WHO_AM_I_MPU9250, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: Read register  err_code:  %x.", err_code);
    APP_ERROR_CHECK(err_code);

    //Check connection
    if(data[0] == 0x71) {
      NRF_LOG_INFO("MPU9250 Connected"); 
    }
    else {
      NRF_LOG_INFO("MPU9250 Not connected"); 
      return MPU9250_NO_CONNECTION;
    }

    //Clear PWR_MGMT_1 Reg to wake up device

     //Read 1 byte from PWR_MGMT_1 register
    data_len = 1;
    err_code = nrf_drv_mpu_read_registers(PWR_MGMT_1, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: PWR_MGMT_1:  %x.", data[0]);
    APP_ERROR_CHECK(err_code);
                  
    err_code = nrf_drv_mpu_write_registers(PWR_MGMT_1, 0x00, 1);
    APP_ERROR_CHECK(err_code);

    //Read 1 byte from PWR_MGMT_1 register
    data_len = 1;
    err_code = nrf_drv_mpu_read_registers(PWR_MGMT_1, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: PWR_MGMT_1:  %x.", data[0]);
    APP_ERROR_CHECK(err_code);

    //Set sensitivity for ACC and Gyro

    //Set Gyro Config
    data_len = 1;
    err_code = nrf_drv_mpu_read_registers(GYRO_CONFIG, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: GYRO_CONFIG:  %x.", data[0]);
    APP_ERROR_CHECK(err_code);

    //Set low sensitivity
    data[0] = data[0] & 0b11100111;
    
    err_code = nrf_drv_mpu_write_registers(GYRO_CONFIG, data, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_mpu_read_registers(GYRO_CONFIG, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: GYRO_CONFIG:  %x.", data[0]);
    APP_ERROR_CHECK(err_code);



    //Set ACCL Config
    data_len = 1;
    err_code = nrf_drv_mpu_read_registers(ACCEL_CONFIG, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: ACCEL_CONFIG:  %x.", data[0]);
    APP_ERROR_CHECK(err_code);

    //Set low sensitivity
    data[0] = data[0] | 0b00011000;
    
    err_code = nrf_drv_mpu_write_registers(ACCEL_CONFIG, data, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_mpu_read_registers(ACCEL_CONFIG, data, data_len);
    NRF_LOG_INFO("From mpu9250_wake: ACCEL_CONFIG:  %x.", data[0]);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

/**
 * @brief Function for reading MPU9250 sensors.
 */
void read_mpu_sensors(mpu9250_sensor_values *sensor_values) {

    ret_code_t err_code;
    uint32_t data_len = MPU_TWI_BUFFER_SIZE;
    uint8_t data[MPU_TWI_BUFFER_SIZE] = {0};

    //Read 14 bytes from ACCEL_XOUT_H addr 0x3B to GYRO_ZOUT_L addr 0x48
    err_code = nrf_drv_mpu_read_registers(ACCEL_XOUT_H, data, data_len);
    APP_ERROR_CHECK(err_code);

    /* For conveinience this comment is put here
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
    */
    
    //Convert ACCL values
    sensor_values->accl_X = (data[0]<<8) + data[1];
    sensor_values->accl_Y = (data[2]<<8) + data[3];
    sensor_values->accl_Z = (data[4]<<8) + data[5];

    //Convert TEMP value
    sensor_values->temp = (data[6]<<8) + data[7];

    //Convert GYRO values
    sensor_values->gyro_X = (data[8]<<8) + data[9];
    sensor_values->gyro_Y = (data[10]<<8) + data[11];
    sensor_values->gyro_Z = (data[12]<<8) + data[13];


}


