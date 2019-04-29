#ifndef functions_h
#define functions_h
#include "mpu9250.h"
//____________________________________functions____________________________________________
int cATAN2(short real, short imag, short N);
short atan2FC(short y,short x); 
 
//_____________________________________Main_loop___________________________________________
/* Struct for storing MPU9250 sensor values */


void process_loop_fixed(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation);   
void process_loop_fixed_asm(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation); 
void process_loop_float(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation); 

int step_detect(int freq_bin, int bin_start, int bin_stop, int acc_value, int detect_threshold);
short dft_float(float x_new);

#endif