#ifndef GAUSSIAN_NOISE_H
#define GAUSSIAN_NOISE_H

class gaussian_noise
{
public:
    double s20;
    double dx_ln(double y, double x, double sigma_x); // d/dx ln P(y|x)
    double dx2_ln(double y, double x, double sigma_x); // d2/dx2 ln P(y|x)
    gaussian_noise(double s20);
};

#endif // GAUSSIAN_NOISE_H
