#include "fft.h"

#include <cmath>
#include <stdexcept>

DctCalculator::DctCalculator(size_t width, std::vector<double> *input, std::vector<double> *output)
    : width_(width), input_(input->data()), output_(output->data()) {
    if (input->size() != width * width) {
        throw std::invalid_argument("Wrong size of input!");
    }

    if (output->size() != width * width) {
        throw std::invalid_argument("Wrong size of output!");
    }
}

void DctCalculator::Inverse() {
    for (size_t i = 0; i < width_ * width_; ++i) {
        if (i < width_) {
            input_[i] /= (1.0 / std::sqrt(2));
        }

        if (i % width_ == 0) {
            input_[i] /= (1.0 / std::sqrt(2));
        }
    }

    fftw_plan plan = fftw_plan_r2r_2d(width_, width_, input_, output_, FFTW_REDFT01, FFTW_REDFT01,
                                      FFTW_ESTIMATE);

    fftw_execute(plan);

    fftw_destroy_plan(plan);

    fftw_cleanup();

    for (size_t i = 0; i < width_ * width_; ++i) {
        output_[i] /= 16;
    }
}