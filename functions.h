#ifndef functions_h
#define functions_h
#include "mpu9250.h"
//____________________________________functions____________________________________________
int cATAN2(short real, short imag, short N);

 
//_____________________________________Main_loop___________________________________________
/* Struct for storing MPU9250 sensor values */


void process_loop_fixed(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation);   
void process_loop_fixed_asm(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation); 
void process_loop_float(mpu9250_sensor_values sensor_values, MPU9250_orientation * mpu_orientation); 



#endif