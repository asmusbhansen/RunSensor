#include "mpu9250.h"
#include <math.h>
     

      
 void process_loop_float(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation){
      //Complementary filter

      float filter_constant = 0.9882;
      static float xz_angle = 0;
      static float yz_angle = 0;

      float xz_angle_acc = atan2((float)sensor_values.accl_X,(float)sensor_values.accl_Z)*180/PI_M;
      float yz_angle_acc = atan2((float)sensor_values.accl_Y,(float)sensor_values.accl_Z)*180/PI_M;

      float gyro_Y = (float)(sensor_values.gyro_Y) / GYRO_SCALE;
      float gyro_X = (float)(sensor_values.gyro_X) / GYRO_SCALE;

      xz_angle = filter_constant*(xz_angle + gyro_Y * UPDATE_LOOP_DT/1000 ) + (1-filter_constant)*xz_angle_acc;
      yz_angle = filter_constant*(yz_angle + gyro_X * UPDATE_LOOP_DT/1000 ) + (1-filter_constant)*yz_angle_acc;

      mpu_orientation->mpu_xz_angle = (int16_t)yz_angle;
      mpu_orientation->mpu_yz_angle = (int16_t)xz_angle;





      static int counter = 0;
      if (counter >= 100) {

        //NRF_LOG_INFO("Max DFT idx: %d", max_dft_idx);
        NRF_LOG_INFO("COMP XZ Angle: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(xz_angle));
        NRF_LOG_INFO("Gyro X: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(gyro_X));
        //NRF_LOG_INFO("COMP XZ Angle: %d", xz_angle_f / (int32_t)(32768 / 180));
        //NRF_LOG_INFO("Angle is XZ %d", (int)xz_angle);
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