#pragma once

#include <vector>
#include <stdint.h>


/**
 * @brief delay signal
 * @todo: template class
 */

class DelayFilt
{
private:
    struct Signal {
        int64_t t;
        double x;
        Signal() : t(0), x(0.0) {}
    };

    std::vector<Signal> _arr;
    int64_t             _delay;
    int                 _pointer;

public:
    DelayFilt();
    void init(int64_t const& delay_usec, int buf_size = 32);
    DelayFilt(int64_t const& delay_usec, int buf_size = 32);
    double process(int64_t const& t_usec, double const& x);
};
