#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/threads.h>
#include <fstream>
#include <butterfly_robot/servo.h>
#include <utils/filters.h>


int feedback_loop(Json::Value const& jscfg)
{
    set_thread_rt_priotiy(-1, 90);

    auto servo = Servo::capture_instance();
    servo->init(jscfg);
    servo->start();

    int64_t t;
    double theta, dtheta;
    bool stop = false;

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
        printf("t = %ld, theta = %f, dtheta = %f, torque = %f\n", t, theta, dtheta, torque);
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

