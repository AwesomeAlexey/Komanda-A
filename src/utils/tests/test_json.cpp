#include <utils/json_bspline.h>

int main(int argc, char const* argv[])
{
    Json::Value cfg = json_load("configs/feedback.json");
    Json::Value fb = json_get(cfg, "sliding_controller");

    Spline<double, 4> x;
    json_get(fb, "traj", x);

    Spline<double, 1> u;
    json_get(fb, "u", u);

    Spline<double, 3> K;
    json_get(fb, "K", K);

    Spline<double, 3> B;
    json_get(fb, "B", B);

    Spline<double, 3> n;
    json_get(fb, "n", n);

    return 0;
}
