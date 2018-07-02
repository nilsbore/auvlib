#include <sparse_gp/octave_convenience.h>

octave_convenience::octave_convenience() : std::stringstream()
{
    *this << "octave --eval \"";
}

// this should be templatized
void octave_convenience::append_vector(const std::vector<double>& y)
{
    *this << "[";
    for (const double& d : y) {
        *this << d << " ";
    }
    *this << "]";
}

int octave_convenience::eval()
{
    *this << "\"";
    int rtn = system(str().c_str());
    str(std::string());
    *this << "octave --eval \"";
    return rtn;
}

int octave_convenience::eval_plot_vector(const std::vector<double>& y)
{
    *this << "plot(";
    append_vector(y);
    *this << "); pause";
    return eval();
}

int octave_convenience::eval_plot_vector_pair(const std::vector<double>& yb,
                                              const std::vector<double>& yr)
{
    *this << "plot(";
    append_vector(yb);
    *this << ", 'b'); hold on; plot(";
    append_vector(yr);
    *this << ", 'r'); pause";
    return eval();
}
