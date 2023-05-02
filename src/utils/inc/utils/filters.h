#pragma once

#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <cppmisc/timing.h>
#include "eigen.h"
#include "utils.h"


/**
 * @brief simple diff:
 *  (x - x_prev) / (t - t_prev)
*/
class EulerDiff
{
private:
    double x0;
    double dx;
    int64_t t0_usec;
    double epsilon;

public:
    EulerDiff(double epsilon = 1e-8) : 
        epsilon(epsilon)
    {
        x0 = 0;
        t0_usec = 0;
    }

    inline double process(int64_t const& t_usec, double const& x)
    {
        if (fabs(x - x0) > epsilon)
        {
            dx = (x - x0) * 1e+6 / (t_usec - t0_usec);
            x0 = x;
            t0_usec = t_usec;
        }

        return dx;      
    }
};
