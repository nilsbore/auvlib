#include <sparse_gp/probit_noise.h>

#include <math.h> // erf

probit_noise::probit_noise(double s20) : s20(s20)
{

}

// d/dx ln P(y|x)
double probit_noise::dx_ln(double y, double x, double sigma_x)
{
    double sigma = sqrt(s20 + sigma_x);
    double z = y*x/sigma;
    double ef = erf(z)/(2.0f*sqrt(2.0f));
    double efprim = exp(-z*z/2)/sqrt(2.0f*M_PI);
    return y/sigma*efprim/ef;
}

// d2/dx2 ln P(y|x)
double probit_noise::dx2_ln(double y, double x, double sigma_x)
{
    double sigma2 = s20 + sigma_x;
    double sigma = sqrt(sigma2);
    double z = y*x/sigma;
    double ef = erf(z)/(2.0f*sqrt(2.0f));
    double efprim = exp(-z*z/2.0f)/sqrt(2.0f*M_PI);
    double efprimprim = -z*efprim;
    double first = efprim/ef;
    return (efprimprim/ef - first*first)/sigma2;
}
