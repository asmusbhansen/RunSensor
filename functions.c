#include <math.h>

short atan_val[10] = {8192, 4836, 2555, 1297, 651,326,163, 81, 41,20};
short sine[60] = {0x0,0xC9,0x192,0x25B,0x324,0x3ED,0x4B6,0x57F,0x648,0x711,0x7D9,0x8A2,0x96B,0xA33,0xAFB,0xBC4,0xC8C,0xD54,0xE1C,0xEE4,0xFAB,0x1073,0x113A,0x1201,0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,0x18F9,0x19BE,0x1A83,0x1B47,0x1C0C,0x1CD0,0x1D93,0x1E57,0x1F1A,0x1FDD,0x209F,0x2162,0x2224,0x22E5,0x23A7,0x2467,0x2528,0x25E8,0x26A8,0x2768,0x2827,0x28E5,0x29A4,0x2A62,0x2B1F,0x2BDC,0x2C99,0x2D55};
short cose[60] = {0x8000,0x7FFF,0x7FFE,0x7FFA,0x7FF6,0x7FF1,0x7FEA,0x7FE2,0x7FD9,0x7FCE,0x7FC2,0x7FB5,0x7FA7,0x7F98,0x7F87,0x7F75,0x7F62,0x7F4E,0x7F38,0x7F22,0x7F0A,0x7EF0,0x7ED6,0x7EBA,0x7E9D,0x7E7F,0x7E60,0x7E3F,0x7E1E,0x7DFB,0x7DD6,0x7DB1,0x7D8A,0x7D63,0x7D3A,0x7D0F,0x7CE4,0x7CB7,0x7C89,0x7C5A,0x7C2A,0x7BF9,0x7BC6,0x7B92,0x7B5D,0x7B27,0x7AEF,0x7AB7,0x7A7D,0x7A42,0x7A06,0x79C9,0x798A,0x794A,0x790A,0x78C8,0x7885,0x7840,0x77FB,0x77B4};

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
short dft_fixed(short x_new){
    x_new = x_new >> 10;
    static short short_dft_r[50];
    static short short_dft_i[50];
    static short samples[1024];
    static short new_idx;
    static short old_idx;

    short temp_s;
    short temp_1;
    short temp_2;
    short dom_freq_indx = 0;
    int dom_freq = 0;
    int abs = 0;

    if(new_idx == 1024){
        new_idx = 0;
    }

    if(new_idx == 1023){
        old_idx = 0;
    } else {
        old_idx = new_idx + 1;
    }

    samples[new_idx] = x_new;

    for(short k = 0;k < 50;k++){
        temp_s = short_dft_r[k] + samples[new_idx] - samples[old_idx];

        temp_1 = (temp_s * cose[k]) >> SHIFT;
        temp_2 = (short_dft_i[k] * sine[k]) >> SHIFT;
        short_dft_r[k] = temp_1 + temp_2;

        temp_1 = (temp_s * sine[k]) >> SHIFT;
        temp_2 = (short_dft_i[k] * cose[k]) >> SHIFT;
        short_dft_i[k] = temp_2 - temp_1;

        
        abs = (((int)short_dft_r[k]*(int)short_dft_r[k]) >> 16) + (((int)short_dft_i[k]*(int)short_dft_i[k]) >> 16);
            
        if(abs > dom_freq){
            dom_freq = abs;
            dom_freq_indx = k;
        }
        
    }
    new_idx++;

    if (abs > 20){
      return dom_freq_indx;
    } else {
      return 0;
    }

}