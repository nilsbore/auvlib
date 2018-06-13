#ifndef OCTAVE_CONVENIENCE_H
#define OCTAVE_CONVENIENCE_H

#include <stdlib.h>
#include <sstream>
#include <vector>

class octave_convenience : public std::stringstream
{
public:
    void append_vector(const std::vector<double>& y);
    int eval_plot_vector(const std::vector<double>& y);
    int eval_plot_vector_pair(const std::vector<double>& yb,
                              const std::vector<double>& yr);
    int eval();
    octave_convenience();
};

#endif // OCTAVE_CONVENIENCE_H
