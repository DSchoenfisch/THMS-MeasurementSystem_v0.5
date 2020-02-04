/**************************************************************************/
/*!
    @file     Linear_Regression.cpp
    @author   D. Schoenfisch (HS-KL.de)

*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Linear_Regression.h"
#include "math.h"

Linear_Regression::Linear_Regression(uint16_t iterations, double alpha) 
{
   m_alpha = alpha;
   m_iterations = iterations;
}

void Linear_Regression::calculate_simpleLR(float x[], float y[], uint16_t array_length, float *b1, float *b0, float *Rpow2) {
    float xm = 0;
    float ym = 0;
    for (uint16_t i = 0; i < array_length; i++) {
        xm += x[i];
        ym += y[i];
    }
    xm = xm/array_length;
    ym = ym/array_length;
    float Cov = 0;   
    float Var = 0;   
    for (uint16_t i = 0; i < array_length; i++) {
        Cov += (x[i]-xm)*(y[i]-ym);
        Var += pow(x[i]-xm,2);
    }
    *b1 = (Cov/Var);
    *b0 = ym - (*b1)*xm;
    float SQR = 0;
    float SQT = 0;
    for (uint16_t i = 0; i < array_length; i++) {
        SQT += pow(y[i]-ym,2);
        float yp =(*b0) + ((*b1) * x[i]);
        SQR += pow(y[i]-yp,2);
    }
    *Rpow2 = 1 - SQR/SQT;
}
