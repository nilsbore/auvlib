#ifndef PROBIT_NOISE_H
#define PROBIT_NOISE_H

class probit_noise
{
public:
    double s20;
    double dx_ln(double y, double x, double sigma_x); // d/dx ln P(y|x)
    double dx2_ln(double y, double x, double sigma_x); // d2/dx2 ln P(y|x)
    probit_noise(double s20);
};

#endif // PROBIT_NOISE_H
