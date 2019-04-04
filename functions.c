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