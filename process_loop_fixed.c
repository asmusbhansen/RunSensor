#include "mpu9250.h"
int BIN_SCALE = 15;

      
      
 void process_loop_fixed(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation){
      //Complementary filter

          
          static int16_t xz_angle_f = 0;
          static int16_t yz_angle_f = 0;

          int16_t filter_constant_1 = 0x799A; // 0.95
          int16_t filter_constant_2 = 0x666; // (1-0.95)
          int32_t mult_temp_1 = 0;
          int32_t mult_temp_2 = 0;
  

          int16_t xz_angle_acc_f = cATAN2(sensor_values.accl_Z, sensor_values.accl_X, 5);
          int16_t yz_angle_acc_f = cATAN2(sensor_values.accl_Z, sensor_values.accl_Y, 5);
      
                //Convert gyro rate to
          int16_t gyro_Y_f = ((-1) * sensor_values.gyro_Y * 32768) / (33 * 180 * 200);
          int16_t gyro_X_f = ((-1) * sensor_values.gyro_X * 32768) / (33 * 180 * 200);
           
          mult_temp_1 = filter_constant_1 * (xz_angle_f + gyro_Y_f ) ; 
          mult_temp_2 = (filter_constant_2) * (xz_angle_acc_f);


          xz_angle_f = mult_temp_1 / ( 1 << BIN_SCALE ) + mult_temp_2 / ( 1 << BIN_SCALE );
  

          mult_temp_1 = (int16_t)(filter_constant_1 * ( yz_angle_f + gyro_X_f )) << 15; 
          mult_temp_2 = (int16_t)(filter_constant_2 * yz_angle_acc_f);

          yz_angle_f = (int16_t)(mult_temp_1 / ( 1 << BIN_SCALE ) + mult_temp_2 / ( 1 << BIN_SCALE ));


      
      mpu_orientation->mpu_xz_angle = (int16_t)(xz_angle_f / (int32_t)(32768 / 180));
      mpu_orientation->mpu_yz_angle = (int16_t)(yz_angle_f / (int32_t)(32768 / 180));

      static int16_t counter = 0;
      if (counter >= 100) {

        //NRF_LOG_INFO("Max DFT idx: %d", max_dft_idx);
        //NRF_LOG_INFO("COMP XZ Angle: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(xz_angle_acc));
        NRF_LOG_INFO("COMP XZ Angle: %d", xz_angle_f / (int32_t)(32768 / 180));


        counter = 0;
      }
      counter++;
}