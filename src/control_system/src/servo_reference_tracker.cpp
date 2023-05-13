#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/threads.h>
#include <cppmisc/signals.h>
#include <butterfly_robot/servo.h>
#include <utils/filters.h>
#include <utils/csv_logger.h>
#include <math.h>

template<typename T>
T sign(T val, double a){
    return val / (abs(val) + a);
}

int feedback_loop(Json::Value const& jscfg)
{
    auto loggercfg = json_get(jscfg, "logger");
    auto path = json_get<std::string>(loggercfg, "saveto");
    CSVLogger logger(path, "t=%f,theta=%f,dtheta=%f,torque=%f");

    auto servo = Servo::capture_instance();
    servo->init(jscfg);

    int64_t t;
    double theta, dtheta;
    bool stop = false;

    auto stop_handler = [&stop]() { stop = true; };
    SysSignals::instance().set_sigint_handler(stop_handler);
    SysSignals::instance().set_sigterm_handler(stop_handler);

    set_thread_rt_priotiy(-1, 50);
    servo->start();

    while (!stop)
    {
        theta = 0;
        dtheta = 0;
        auto status = servo->get_state(t, theta, dtheta, true);
        if (failed(status))
        {
            err_msg("received corrupted packet");
            return -1;
        }

        double torque = -0.5 * std::clamp(theta, -0.5, 0.5) - 0.1 * dtheta;
        auto J = 0.891e-3;
        auto a = 0.001;
        auto c = 1.;
        auto alpha = 1.;
        auto cd = 2.15e-3;
        auto cv = 5.67e-2;
        auto beta = 0.0;
        auto gamma = 1.;
        auto delta = 1.;
        auto T0 = 0.0;
        auto maxRot = 5.;
        auto maxRef2 = gamma * (delta^2);
//        auto ref = 1.;
//        auto dref = 0.;

        auto ref = T0 + beta*t + gamma*sin(delta*t);
        auto dref = beta + gamma*delta*cos(delta*t);

        double p1 = (-1 / c) * (dtheta - dref);
        double p2 = (alpha / c) + (cd / J) + (cv / J)*maxRot + maxRef2;
        double p22 = p2*sign(theta - ref + c*(dtheta - dref))
        double torque = J * (p1 + p22);

        torque = std::clamp(torque, -0.1, 0.1);
        servo->set_torque(torque);
        logger.write(usec_to_sec(t), theta, dtheta, torque);
    }

    servo->set_torque(0.0);
    servo->stop();
    return 0;
}

int main(int argc, char const* argv[])
{
    Arguments args({
        Argument("-c", "config", "path to json config file", "", ArgumentsCount::One)
    });

    int status = 0;

    try
    {
        auto&& m = args.parse(argc, argv);
        Json::Value const& cfg = json_load(m["config"]);
        traces::init(json_get(cfg, "traces"));
        status = feedback_loop(cfg);
    }
    catch (std::exception const& e)
    {
        err_msg(e.what());
        status = -1;
    }
    catch (...)
    {
        err_msg("Unknown error occured");
        status = -1;
    }

    return status;
}

