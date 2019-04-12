#include <math.h>

short atan_val[10] = {8192, 4836, 2555, 1297, 651,326,163, 81, 41,20};

//_____________________________________cATAN2___________________________________________
int cATAN2(short real, short imag, short N) {

  short theta;
  short alfa;
  if (imag > 0 && real > 0) { //First quadrant
    alfa = 0;
    theta = 32767;                   //1-LSB
  } else if (imag > 0 && real < 0) { //Second quadrant
    alfa = 32767;
    theta = -32767;
  } else if (imag < 0 && real < 0) { //Third quadrant
    alfa = -32767;
    theta = 32767;
  } else if (imag < 0 && real > 0) { //Fourth quadrant
    alfa = 0;
    theta = -32767;
  } else {
    return 0;
  }

  real = abs(real);
  imag = abs(imag);

  short current_theta = 0;
  short temp = 0;


  for (int i = 0; i < N; i++) {
    if (imag < 0) { //negativ angle
      current_theta = current_theta - atan_val[i];

      temp = real - (imag >> i);
      imag = imag + (real >> i);
      real = temp;
    } else { //positiv angle
      current_theta = current_theta + atan_val[i];

      temp = real + (imag >> i);
      imag = imag - (real >> i);
      real = temp;
    }
  }
  short theta_est;
  if (theta == 32767) {
    theta_est = current_theta + alfa;
    return theta_est;
  } else if (theta == -32767) {
    theta_est = -current_theta + alfa;
    return theta_est;
  }
}


//________________________________________________________________________________

int step_detect(int freq_bin, int bin_start, int bin_stop, mpu9250_sensor_values sensor_values, int detect_threshold)
{

  static int max_acc_x = 0;
  static int max_acc_y = 0;
  static int max_acc_z = 0;
  
  static int dominant_acc_axis = 0;

  //If were not in the correct frequency area, no step is detected.
  if(freq_bin < bin_start || freq_bin > bin_stop)
  {
    return 0;
  }
   
  //Decrement maximum amplitude
  if(max_acc_x > 0)
  {
    max_acc_x--;
  }

  if(max_acc_y > 0)
  {
    max_acc_y--;
  }


  if(max_acc_z > 0)
  {
    max_acc_z--;
  }

  //If the current maximum acceleration amplitude is lower than the measurement.
  if(max_acc_x < abs(sensor_values.accl_X))
  {
    max_acc_x = abs(sensor_values.accl_X);
  }
  
  if(max_acc_y < abs(sensor_values.accl_Y))
  {
    max_acc_Y = abs(sensor_values.accl_Y);
  }

  if(max_acc_Z < abs(sensor_values.accl_Z))
  {
    max_acc_Z = abs(sensor_values.accl_Z);
  }

  //See which axis has the largest acceleration maximum
  if(max_acc_x > max_acc_y && max_acc_x > max_acc_z)
  {
    //dominant_acc_axis = 1;
    if(sensor_values.accl_X > (float)max_acc_x * detect_threshold)
    {
      return 1;
    }
  }
  else if(max_acc_y > max_acc_x && max_acc_y > max_acc_z)
  {
    //dominant_acc_axis = 2;
    if(sensor_values.accl_Y > (float)max_acc_y * detect_threshold)
    {
      return 1;
    }
  }
  else
  {
    //dominant_acc_axis = 3;
    if(sensor_values.accl_Z > (float)max_acc_Z * detect_threshold)
    {
      return 1;
    }
  }



  
  



}