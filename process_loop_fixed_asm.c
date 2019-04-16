#include "mpu9250.h"
extern int compFilt(short, short, short);
      
extern short pATAN2(short, short);

      
 void process_loop_fixed_asm(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation){

      static int16_t xz_angle_f = 0;
      static int16_t yz_angle_f = 0;

      int gyro_scale = (int)GYRO_SCALE;
      //Complementary filter


      //ACC Angle virker i fixed point
      // Approximation: 1/PI_M * 32768 = 10431 - The approximation deviates from the true value by: 45 - (atan(1)  * 10431) * pi/(2^15) * 180/pi = 0.0026 degrees

      //int16_t xz_angle_acc_f = cATAN2(sensor_values.accl_X, sensor_values.accl_Z,CORDIC_ITERATIONS);
      //int16_t yz_angle_acc_f = cATAN2(sensor_values.accl_Y, sensor_values.accl_Z,CORDIC_ITERATIONS);
      int16_t xz_angle_acc_f = pATAN2(sensor_values.accl_X, sensor_values.accl_Z);
      int16_t yz_angle_acc_f = pATAN2(sensor_values.accl_Y, sensor_values.accl_Z);
      
      //short xz_angle_acc_f_1 = (int32_t)(atan2((float)sensor_values.accl_X,(float)sensor_values.accl_Z) * 10431);

      //Convert gyro rate to
      int16_t gyro_Y_f = (sensor_values.gyro_Y * 32768) / (gyro_scale * 180 * 200);
      int16_t gyro_X_f = (sensor_values.gyro_X * 32768) / (gyro_scale * 180 * 200);

      //xz_angle_f = (int16_t)(mult_temp_1 / ( 1 << BIN_SCALE ) + mult_temp_2 / ( 1 << BIN_SCALE ));
      xz_angle_f = compFilt(gyro_Y_f, xz_angle_acc_f, xz_angle_f);
      yz_angle_f = compFilt(gyro_X_f, yz_angle_acc_f, yz_angle_f);

      
      //mpu_orientation->mpu_xz_angle = (int16_t)(xz_angle_f / (int32_t)(32768 / 180));
      //mpu_orientation->mpu_yz_angle = (int16_t)(yz_angle_f / (int32_t)(32768 / 180));

      mpu_orientation->mpu_xz_angle = (int16_t)(xz_angle_f);
      mpu_orientation->mpu_yz_angle = (int16_t)(yz_angle_f);


      static int16_t counter = 0;
      if (counter >= 100) {

        //NRF_LOG_INFO("Max DFT idx: %d", max_dft_idx);
        //NRF_LOG_INFO("COMP XZ Angle: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(xz_angle_acc));
        NRF_LOG_INFO("COMP YZ Angle: %d", yz_angle_f / (int32_t)(32768 / 180));
        //NRF_LOG_INFO("Angle is XZ %d", xz_angle_f);
        //NRF_LOG_INFO("alfa %d", alfa1);
        //NRF_LOG_INFO("ACC XZ Angle: %d", xz_angle_acc_f / (int32_t)(32768 / 180));
        //NRF_LOG_INFO("Mult temp 1 shifted: %d", mult_temp_1>>BIN_SCALE);
        //NRF_LOG_INFO("Mult temp 2 shifted: %d", mult_temp_2>>BIN_SCALE);

        //NRF_LOG_INFO("COMP XZ Angle: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(xz_angle_acc));
        //NRF_LOG_INFO("Gyro X Fixed: %d", (int32_t)(gyro_Y_f * 180 * 200) / 32768);

        counter = 0;
      }
      counter++;
}