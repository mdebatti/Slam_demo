// Functions.cpp


#include "functions.h"


double a_sub(const double& a1,const double& a2)
{
    double a = a1 - a2;

    // get to within 2pi of the right answer, the int conversion
    // just drops the digits after the decimal point, thus acting
    // the matlab fix function
    a = a - (2.0*PI*(int)(a / (2.0*PI)));

    // then leave a between +/- pi
    if(a > PI)
        a = a - (2.0*PI);

    if(a < -PI)
        a = a + (2.0*PI);

    return a;
}

double  a_add(const double& a1,const double& a2)
{
    double a = a1 + a2;

    // get to within 2pi of the right answer, the int conversion
    // just drops the digits after the decimal point, thus acting
    // the matlab fix function
    a = a - (2.0*PI*(int)(a / (2.0*PI)));

    // then leave a between +/- pi
    if(a > PI)
        a = a - (2.0*PI);
    if(a < -PI)
        a = a + (2.0*PI);

    return a;
}


