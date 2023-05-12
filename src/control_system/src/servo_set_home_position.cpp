#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/threads.h>
#include <cppmisc/signals.h>
#include <butterfly_robot/servo.h>
#include <utils/filters.h>
#include <utils/csv_logger.h>


int feedback_loop(Json::Value const& jscfg)
{
    auto loggercfg = json_get(jscfg, "logger");
    auto path = json_get<std::string>(loggercfg, "saveto");
    CSVLogger logger(path, "t = %ld, theta = %f, dtheta = %f, torque = %f");

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
        torque = std::clamp(torque, -0.1, 0.1);
        servo->set_torque(torque);
        logger.write(t, theta, dtheta, torque);
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

