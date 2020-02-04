/**************************************************************************/
/*!
    @file     Linear_Regression.h
    @author   D. Schoenfisch (HS-KL.de)

*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
/*=========================================================================
    Config
    -----------------------------------------------------------------------*/
    #define ITERATIONS    (1)           // Number of iterations
    #define ALPHA         (0.001)        // Step length
/*=========================================================================*/

class Linear_Regression
{
protected:
    double      m_alpha;
    uint16_t    m_iterations;

public:
    Linear_Regression(uint16_t iterations = ITERATIONS, double alpha = ALPHA);
    void        calculate_simpleLR(float x[], float y[], uint16_t array_length, float *b1, float *b0, float *Rpow2);

private:
};
