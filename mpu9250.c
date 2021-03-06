#include "mpu9250.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "functions.h"
#include "ble_run.h"

/* Indicates if operation on TWI has ended. */


static volatile bool m_xfer_done = false;

/* TX Buffer*/
uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

/* TWI instance. */
static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Dummy functions to avoid linker complaints */
void __aeabi_unwind_cpp_pr0(void){};
EXPORT_SYMBOL(__aeabi_unwind_cpp_pr0);

void __aeabi_unwind_cpp_pr1(void){};
EXPORT_SYMBOL(__aeabi_unwind_cpp_pr1);

void __aeabi_unwind_cpp_pr2(void){};
EXPORT_SYMBOL(__aeabi_unwind_cpp_pr2);

/* Number of readings */
array_list_t p_rx_buffer[2 * TWIM_RX_BUF_LENGTH];

/* Declare a simple TX buffer holding the first register in MPU we want to read from. */
uint8_t p_tx_buffer[1] = {ACCEL_XOUT_H}; // Reading accelerometer only

/* Init struct for mpu orientation */
MPU9250_orientation mpu_orientation = {0};

/* PPI Channel for link between timer and DMA */
nrf_ppi_channel_t ppi_channel;

/* Buffer for angular estimates*/
uint16_t xz_angle_buffer_index = 0;
uint16_t yz_angle_buffer_index = 0;
float xz_angle_buffer[DFT_LEN + 1] = {0};
float yz_angle_buffer[DFT_LEN + 1] = {0};

/* DFT Array */
float dft_xz_re[DFT_LEN] = {0};
float dft_xz_im[DFT_LEN] = {0};

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  switch (p_event->type) {
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
uint32_t nrf_drv_mpu_init(void) {
  uint32_t err_code;

  const nrf_drv_twi_config_t twi_mpu_config = {
      .scl = MPU_TWI_SCL_PIN,
      .sda = MPU_TWI_SDA_PIN,
      .frequency = NRF_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi_instance, &twi_mpu_config, twi_handler, NULL);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  nrf_drv_twi_enable(&m_twi_instance);

  NRF_LOG_INFO("TWI Init");

  return NRF_SUCCESS;
}

/**
 * @brief TWI deactivation
 */
void nrf_drv_mpu_deavtivate(void) {

  void nrf_drv_twi_disable(m_twi_instance);
  nrf_drv_twi_uninit(&m_twi_instance);
}

/**
 * @brief Function for reading from MPU9250 register
 */
static int32_t nrf_drv_mpu_read_registers(uint8_t reg, uint8_t *p_data, uint32_t length) {
  uint32_t err_code;
  uint32_t timeout = MPU_TWI_TIMEOUT;

  //Write the address which we want to read from, to the MPU9250
  err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_ADDRESS, &reg, 1, false);
  if (err_code != NRF_SUCCESS)
    return err_code;

  // Wait for transfer finish or timeout
  while ((!m_xfer_done) && --timeout)
    ;
  if (!timeout) {
    return NRF_ERROR_TIMEOUT;
  }
  m_xfer_done = false;

  //Read lenght x 8bit data from the MPU9250
  err_code = nrf_drv_twi_rx(&m_twi_instance, MPU_ADDRESS, p_data, length);
  if (err_code != NRF_SUCCESS)
    return err_code;

  timeout = MPU_TWI_TIMEOUT;
  while ((!m_xfer_done) && --timeout)
    ;
  if (!timeout) {
    return NRF_ERROR_TIMEOUT;
  }
  m_xfer_done = false;

  return err_code;
}

/**
 * @brief Function for writing MPU9250 register
 */
static uint32_t nrf_drv_mpu_write_registers(uint8_t reg, uint8_t *p_data, uint32_t length) {
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

  while ((!m_xfer_done) && --timeout)
    ;
  if (!timeout) {
    return NRF_ERROR_TIMEOUT;
  }
  m_xfer_done = false;

  return err_code;
}

/**
 * @brief Function for waking up MPU9250.
 */
uint32_t mpu9250_wake(void) {
  ret_code_t err_code;
  uint32_t data_len = 0;
  uint8_t data[1] = {0};

  //Read 1 byte from WHO_AM_I_MPU9250 register to verify connection
  data_len = 1;
  err_code = nrf_drv_mpu_read_registers(WHO_AM_I_MPU9250, data, data_len);
  NRF_LOG_INFO("From mpu9250_wake: WHO_AM_I:  %x.", data[0]);
  NRF_LOG_INFO("From mpu9250_wake: Read register  err_code:  %x.", err_code);
  //while(1);
  APP_ERROR_CHECK(err_code);

  //Check connection
  if (data[0] == 0x71) {
    NRF_LOG_INFO("MPU9250 Connected");
  } else {
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
  //data[0] = data[0] & 0b11100111;
  //data[0] = data[0] | 0b00001000;

  //Set low sensitivity
  data[0] = 0x8;

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
  sensor_values->accl_X = (data[0] << 8) + data[1];
  sensor_values->accl_Y = (data[2] << 8) + data[3];
  sensor_values->accl_Z = (data[4] << 8) + data[5];

  //Convert TEMP value
  sensor_values->temp = (data[6] << 8) + data[7];

  //Convert GYRO values
  sensor_values->gyro_X = (data[8] << 8) + data[9];
  sensor_values->gyro_Y = (data[10] << 8) + data[11];
  sensor_values->gyro_Z = (data[12] << 8) + data[13];
}

/**@brief Function for initializing TWI DMA transfer.
 */
void mpu_twi_dma_init() {
  ret_code_t err_code;

  err_code = nrf_drv_ppi_init();
  if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED)) {
    return NRF_ERROR_NO_MEM;
  }
  if (err_code == NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
    NRF_LOG_INFO("PPI Already initialized.");
  } else {
    NRF_LOG_INFO("PPI initialized.");
  }

  err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("PPI Channel allocated.");

  // GPIOTE init
  nrf_gpio_cfg_output(LED_3); //GPIOTE for debug purposes
  // Configure GPIOTE channel 0 to toggle the PWM pin state
  nrf_gpiote_task_configure(0, LED_3, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW); //GPIOTE for debug purposes

  NRF_TIMER1->TASKS_STOP = 1;                                                       // Stop timer
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;                                         // Set timer 1 to timer mode
  NRF_TIMER1->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos); //Timer should be 16 bit
  NRF_TIMER1->PRESCALER = 4;                                                        // 1us resolution
  NRF_TIMER1->TASKS_CLEAR = 1;                                                      // Clear timer
  NRF_TIMER1->CC[0] = UPDATE_LOOP_DT * 1000;                                        //Timer compare every 10000us(10ms) for a sample frequency = 100Hz
  //NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos; // Enable interrupt for compare 0 on timer1
  NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos); //Timer is cleared on compare

  // Disable the TWIM module while we reconfigure it
  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
  NRF_TWIM0->SHORTS = 0;
  NVIC_DisableIRQ(SPI0_TWI0_IRQn);
  NVIC_ClearPendingIRQ(SPI0_TWI0_IRQn);

  NRF_TWIM0->PSEL.SCL = MPU_TWI_SCL_PIN;
  NRF_TWIM0->PSEL.SDA = MPU_TWI_SDA_PIN;
  NRF_TWIM0->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;

  // Load TWI TX buffer into TWI module. Set number of bytes to write pr transfer, max count, to one.
  // Disable the EasyDMA list functionality for TWI TX.
  NRF_TWIM0->TXD.PTR = (uint32_t)&p_tx_buffer;
  NRF_TWIM0->TXD.MAXCNT = 1;
  NRF_TWIM0->TXD.LIST = TWIM_TXD_LIST_LIST_Disabled << TWIM_TXD_LIST_LIST_Pos;

  // Point to TWI RX buffer. Set number of bytes to read pr transfer, max count, to TWIM_RX_BUF_WIDTH.
  // Disable the EasyDMA list functionality for TWI TX
  NRF_TWIM0->RXD.PTR = (uint32_t)&p_rx_buffer;
  NRF_TWIM0->RXD.MAXCNT = TWIM_RX_BUF_WIDTH;
  NRF_TWIM0->RXD.LIST = TWIM_RXD_LIST_LIST_ArrayList << TWIM_RXD_LIST_LIST_Pos;

  // Make sure that MPU address is set
  NRF_TWIM0->ADDRESS = MPU_ADDRESS;
  // Enable shortcuts that starts a read right after a write and sends a stop condition after last TWI read
  NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
                      (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);

  //Gives the assigned PPI channel event end point(EEP) NRF_TIMER1->EVENTS_COMPARE[0] and Task end point(TEP) NRF_TWIM0->TASKS_STARTTX
  err_code = nrf_drv_ppi_channel_assign(ppi_channel, (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0], (uint32_t)&NRF_TWIM0->TASKS_STARTTX);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("PPI Channel assigned EEP and TEP.");

  err_code = nrf_drv_ppi_channel_fork_assign(ppi_channel, (uint32_t)&NRF_GPIOTE->TASKS_OUT[0]);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("PPI Channel assigned GPIOTE evt as FORK.");

  // Enable the TWIM module
  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
}

/**@brief Function for printing DMA buffer.
 */
void print_dma_buffer() {

  uint32_t rx_counter = (uint32_t)NRF_TWIM0->RXD.PTR;
  uint32_t rx_buffer_offset = (uint32_t)&p_rx_buffer;

  for (int i = 0; i < TWIM_RX_BUF_LENGTH; i++) {

    for (int j = 0; j < TWIM_RX_BUF_WIDTH; j++) {

      if (p_rx_buffer[i].buffer[j] != 0) {
        NRF_LOG_INFO("DMA RAM: %x at buf addr %d, RAM addr: %d.", p_rx_buffer[i].buffer[j], i, rx_counter - rx_buffer_offset);
        p_rx_buffer[i].buffer[j] = 0;
      }
    }
  }
}

/**@brief Function for starting DMA transfer.
 */
void start_twi_dma_transfer() {
  //Enable PPI Channel
  ret_code_t err_code;

  err_code = nrf_drv_ppi_channel_enable(ppi_channel);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("PPI Channel Enabled.");

  // Start timer
  NRF_TIMER1->TASKS_START = 1;
}

/**@brief Function for stopping DMA transfer.
 */
void stop_twi_dma_transfer() {

  //Enable PPI Channel
  ret_code_t err_code;

  err_code = nrf_drv_ppi_channel_disable(ppi_channel);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("PPI Channel Disabled.");

  // Stop timer
  NRF_TIMER1->TASKS_STOP = 1;
}



/**@brief Function for processing mpu data - This is the main processing function for MPU data
 */
void process_mpu_data() {

  ret_code_t err_code;
  uint32_t rx_counter = (uint32_t)NRF_TWIM0->RXD.PTR;
  uint32_t rx_buffer_offset = (uint32_t)&p_rx_buffer;
  uint32_t measurements_available = 0;
  mpu9250_sensor_values sensor_values = {0};


  static int16_t yz_angle_f = 0;  //complementary filter
  static int16_t xz_angle_f = 0; //complementary filter


  float xz_angle_acc = 0;
  float yz_angle_acc = 0;
  int16_t xz_angle_acc_f = 0;
  int16_t yz_angle_acc_f = 0;

  float gyro_X = 0;
  float gyro_Y = 0;
  int16_t gyro_X_f = 0;
  int16_t gyro_Y_f = 0;

  int16_t freq_bin = 0;

  int buffer_offset = 0;


  static int counter = 0;


  //In the first iteration the data should be read intp the array p_rx_buffer from index 0. Before processing the data, we instruct the DMA to load data into the buffer from index TWIM_RX_BUF_LENGTH for processing in next iteration
  //The next iteration will read the data from the offset posistion in the buffer, but first make sure that the next iteration the data will be read into the buffer from position 0.

  //Determine if the buffer is offset
  //The data buffer is split into two part of length TWIM_RX_BUF_LENGTH * TWIM_RX_BUF_WIDTH size each.
  //If the current counter is higher half of the total buffer, the buffer is offset
  if (rx_buffer_offset + sizeof(p_rx_buffer) / 2 < rx_counter) {
    buffer_offset = 1;
  }

  //We want to calculate the samples available. Rx_counter is where the last reading was put into, rx_buffer offset is where the first reading can be put into.
  //If the buffer is offset, we start halfway into the buffer i.e. sizeof(p_rx_buffer)/2 bytes into the buffer.
  //Hence, when calculating the measurements_available we do no consider the first part of the buffer and subtract sizeof(p_rx_buffer
  measurements_available = (rx_counter - rx_buffer_offset) / TWIM_RX_BUF_WIDTH - buffer_offset * sizeof(p_rx_buffer) / 2;

  //NRF_LOG_INFO("Measurements available: %d", measurements_available);

  //Only process data if the buffer is at least half full
  if (measurements_available >= TWIM_RX_BUF_LENGTH / 2) {

    //Instruct the DMA Where to put data next. If the buffer was determined to be offset we want to put data into index 0 from next iteration
    NRF_TWIM0->RXD.PTR = (uint32_t)(rx_buffer_offset + (1 - buffer_offset) * buffer_offset * sizeof(p_rx_buffer) / 2);

    

    //Begin processing the data
    for (int i = 0; i < measurements_available; i++) {

      //When reading data from the buffer in RAM, we offset by half the buffer if the buffer is determined to be offset.
      read_mpu_data_RAM(&sensor_values, i + buffer_offset * TWIM_RX_BUF_LENGTH);
      
<<<<<<< HEAD
=======
      nrf_gpio_pin_set(LED_3);

>>>>>>> a2f36cf11a6a3942f5f9e16e1ec7c6bbe8e43543
      process_loop_fixed(sensor_values, &mpu_orientation);
      //process_loop_fixed_asm(sensor_values, &mpu_orientation);
      //process_loop_float(sensor_values, &mpu_orientation);
      nrf_gpio_pin_clear(LED_3);

      //freq_bin = dft_fixed(mpu_orientation.mpu_yz_angle);
      //nrf_gpio_pin_set(LED_3);
      freq_bin = sDFT(mpu_orientation.mpu_yz_angle);
      //nrf_gpio_pin_clear(LED_3);
      //freq_bin = dft_float(mpu_orientation.mpu_yz_angle);

      if(step_detect(freq_bin, 5, 60, sensor_values.accl_Z, 0.5))
      {

        notify_ble();
      }
    }

    
  }
  
}

/**@brief Function for reading mpu data from RAM.
 */
void read_mpu_data_RAM(mpu9250_sensor_values *sensor_values, uint32_t location) {

  //Convert ACCL values
  sensor_values->accl_X = (p_rx_buffer[location].buffer[0] << 8) + p_rx_buffer[location].buffer[1];
  sensor_values->accl_Y = (p_rx_buffer[location].buffer[2] << 8) + p_rx_buffer[location].buffer[3];
  sensor_values->accl_Z = (p_rx_buffer[location].buffer[4] << 8) + p_rx_buffer[location].buffer[5];

  //Convert TEMP value
  sensor_values->temp = (p_rx_buffer[location].buffer[6] << 8) + p_rx_buffer[location].buffer[7];

  //Convert GYRO values
  sensor_values->gyro_X = (p_rx_buffer[location].buffer[8] << 8) + p_rx_buffer[location].buffer[9];
  sensor_values->gyro_Y = (p_rx_buffer[location].buffer[10] << 8) + p_rx_buffer[location].buffer[11];
  sensor_values->gyro_Z = (p_rx_buffer[location].buffer[12] << 8) + p_rx_buffer[location].buffer[13];
}

/**@brief Function for obtaining MPU orientation struct.
 */
void get_mpu_orientation(MPU9250_orientation *orientation) {

  memcpy(orientation, &mpu_orientation, sizeof(MPU9250_orientation));
}

/** @Brief Function for retrieving a value from a circular buffer*/
float get_value_c_buffer(float *buf, uint16_t buffer_len, uint16_t newest_idx, uint16_t get_idx) {

  //If the newest entry in the circular buffer is at index n and the wanted value is m samples behind we wrap around
  if (get_idx > newest_idx) {
    get_idx -= newest_idx;
    //NRF_LOG_INFO("C buf idx read: %d", buffer_len-get_idx);
    return buf[buffer_len - get_idx];
  } else {
    //NRF_LOG_INFO("C buf idx read: %d", newest_idx-get_idx);
    return buf[newest_idx - get_idx];
  }
}

/** @Brief Function for setting a value in a circular buffer*/
uint16_t set_value_c_buffer(float *buf, uint16_t buffer_len, float val) {

  static uint16_t buffer_index = 0;

  if (buffer_index >= buffer_len) {
    buffer_index = 0;
  }

  //NRF_LOG_INFO("C buf idx write: %d", buffer_index);

  buf[buffer_index] = val;

  buffer_index += 1;

  return buffer_index;
}