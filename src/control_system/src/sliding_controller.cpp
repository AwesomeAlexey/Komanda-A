#include <control_system/sliding_controller.h>
#include <utils/utils.h>


SlidingController::SlidingController()
{
    _P = makemat<3, 4>(
        1., 0., 0., 0.,
        0., 0., 1., 0.,
        0., 0., 0., 1.
    );
}

SlidingController::~SlidingController()
{
}

void SlidingController::load(Json::Value const& cfg)
{
    Json::Value slidingcfg = json_get(cfg, "sliding_controller");
    json_get(slidingcfg, "traj", _x_sp);
    json_get(slidingcfg, "u", _u_sp);

    auto Kcfg = json_get(slidingcfg, "K");
    if (Kcfg.type() == Json::ValueType::arrayValue)
        json_get(slidingcfg, "K", _K);
    else if (Kcfg.type() == Json::ValueType::objectValue)
        json_get(slidingcfg, "K", _K_sp);
    else
        throw_runtime_error("The field sliding_controller.K has incorrect type ", int(Kcfg.type()));
    json_get(slidingcfg, "B", _B_sp);
    json_get(slidingcfg, "n", _n_sp);
    json_get(slidingcfg, "sliding_coef", _sliding_coef);
}

double SlidingController::process(double theta, double phi, double dtheta, double dphi)
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
    RVec3 K = eval_K(phi_0_pi);
    auto x_ref = _x_sp(phi_0_pi);
    Vec3 xi = _P * (x - x_ref);
    xi(0) = normalize_angle(xi(0), -_PI_2, _PI_2);
    double w = _sliding_coef * sign(double(n * xi) * double(n * B));
    double v = K * xi;
    double u = _u_sp(phi_0_pi) + v;
    return u;
}

inline RVec3 SlidingController::eval_K(double phi) const
{
    if (_K_sp.initialized())
        return _K_sp(phi).transpose();
    return _K;
}
