#pragma once

#include <cstddef>
#include <fftw3.h>
#include <vector>

class DctCalculator {
public:
    // input and output are width by width matrices, first row, then
    // the second row.
    DctCalculator(size_t width, std::vector<double> *input, std::vector<double> *output);

    void Inverse();

private:
    size_t width_;
    double *input_, *output_;
};
