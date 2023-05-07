#include <utils/json_bspline.h>
#include <utils/eigen.h>
#include <utils/utils.h>


class SlidingController
{
public:
    SlidingController()
    {
        _P = makemat<3, 4>(
            1., 0., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.
        );
    }

    ~SlidingController()
    {
    }

    void load(Json::Value const& cfg)
    {
        Json::Value fb = json_get(cfg, "sliding_controller");
        json_get(fb, "traj", _x_sp);
        json_get(fb, "u", _u_sp);
        json_get(fb, "K", _K_sp);
        json_get(fb, "B", _B_sp);
        json_get(fb, "n", _n_sp);
        json_get(fb, "sliding_coef", _sliding_coef);
    }

    double process(double theta, double phi, double dtheta, double dphi)
    {
        if (!_initialized)
        {
            _initialized = true;
            _phi_prev = phi;
        }
        phi = std::max(phi, _phi_prev);
        _phi_prev = phi;
        auto x = makevec(theta, phi, dtheta, dphi);
        double phi_0_pi = normalize_angle(phi, 0, _PI);
        RVec3 n = _n_sp(phi_0_pi).transpose();
        Vec3 B = _B_sp(phi_0_pi);
        RVec3 K = _K_sp(phi_0_pi).transpose();
        auto x_ref = _x_sp(phi_0_pi);
        Vec3 xi = _P * (x - x_ref);
        xi(0) = normalize_angle(xi(0), -_PI_2, _PI_2);
        double w = _sliding_coef * sign(double(n * xi) * double(n * B));
        double v = K * xi;
        double u = _u_sp(phi_0_pi) + v;
        return u;
    }

private:
    bool _initialized = false;
    double _phi_prev;
    Mat34 _P;
    Spline<double, 3> _n_sp;
    Spline<double, 3> _B_sp;
    Spline<double, 3> _K_sp;
    Spline<double, 4> _x_sp;
    Spline<double, 1> _u_sp;
    double _sliding_coef;
};
