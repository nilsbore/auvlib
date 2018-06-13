#include <sparse_gp/gaussian_noise.h>

gaussian_noise::gaussian_noise(double s20) : s20(s20)
{

}

// d/dx ln P(y|x)
double gaussian_noise::dx_ln(double y, double x, double sigma_x)
{
    return (y - x)/(s20 + sigma_x);
}

// d2/dx2 ln P(y|x)
double gaussian_noise::dx2_ln(double y, double x, double sigma_x)
{
    return -1.0f/(s20 + sigma_x);
}
