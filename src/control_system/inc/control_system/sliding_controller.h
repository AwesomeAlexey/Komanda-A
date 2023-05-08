#pragma once

#include <utils/eigen.h>
#include <utils/json_bspline.h>


class SlidingController
{
public:
    SlidingController();
    ~SlidingController();
    void load(Json::Value const& cfg);
    double process(double theta, double phi, double dtheta, double dphi);

private:
    bool _initialized = false;
    double _phi_prev;
    Mat34 _P;
    RVec3 _K;
    Spline<double, 3> _n_sp;
    Spline<double, 3> _B_sp;
    Spline<double, 3> _K_sp;
    Spline<double, 4> _x_sp;
    Spline<double, 1> _u_sp;
    double _sliding_coef;

    inline RVec3 eval_K(double phi) const;
};
