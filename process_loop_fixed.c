#include "mpu9250.h"
int BIN_SCALE = 15;

      
      
 void process_loop_fixed(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation){
      //Complementary filter

          
          static int16_t xz_angle_f = 0;
          static int16_t yz_angle_f = 0;

          int gyro_scale = (int)GYRO_SCALE;

          int16_t filter_constant_1 = 0x7d70; // 0.9882
          int16_t filter_constant_2 = 0x28f; // (1-0.9882)
          int32_t mult_temp_1 = 0;
          int32_t mult_temp_2 = 0;
          int64_t gyro_temp = 0;

          
          int16_t xz_angle_acc_f = atan2FC(sensor_values.accl_Z, sensor_values.accl_X);

          int16_t yz_angle_acc_f = atan2FC(sensor_values.accl_Z, sensor_values.accl_Y);
      
          //Convert gyro rate to
          //gyro_temp = (-1) * (int64_t)sensor_values.gyro_Y * 32768;
          //int16_t gyro_Y_f = (int16_t)(gyro_temp / (gyro_scale * 180 * 200));

          int16_t gyro_Y_f = (sensor_values.gyro_Y * 32768) / (gyro_scale * 180 * 200);

          //gyro_temp = (-1) * (int64_t)sensor_values.gyro_X * 32768;
          //int16_t gyro_X_f = (int16_t)(gyro_temp / (gyro_scale * 180 * 200));
          
          int16_t gyro_X_f = (sensor_values.gyro_X * 32768) / (gyro_scale * 180 * 200);

          mult_temp_1 = filter_constant_1 * (xz_angle_f + gyro_Y_f ) ; 
          mult_temp_2 = filter_constant_2 * xz_angle_acc_f;


          xz_angle_f = (int16_t)(mult_temp_1 / ( 1 << BIN_SCALE ) + mult_temp_2 / ( 1 << BIN_SCALE ));
  

          mult_temp_1 = filter_constant_1 * ( yz_angle_f + gyro_X_f ); 
          mult_temp_2 = filter_constant_2 * yz_angle_acc_f;

          yz_angle_f = (int16_t)(mult_temp_1 / ( 1 << BIN_SCALE ) + mult_temp_2 / ( 1 << BIN_SCALE ));
          
      mpu_orientation->mpu_xz_angle = (int16_t)(xz_angle_f / (int32_t)(32768 / 180));
      mpu_orientation->mpu_yz_angle = (int16_t)(yz_angle_f / (int32_t)(32768 / 180));


      static int16_t counter = 0;
      if (counter >= 100) {

        //NRF_LOG_INFO("Max DFT idx: %d", max_dft_idx);
        //NRF_LOG_INFO("COMP XZ Angle: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(xz_angle_acc));
        NRF_LOG_INFO("COMP YZ Angle: %d", yz_angle_f / (int32_t)(32768 / 180));
        //NRF_LOG_INFO("COMP Fixed X Gyro: %d", gyro_X_f);


        counter = 0;
      }
      counter++;
}