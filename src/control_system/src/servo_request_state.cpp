#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/threads.h>
#include <butterfly_robot/servo.h>


int test_1(Json::Value const& jscfg)
{
    auto servo = Servo::capture_instance();
    servo->init(jscfg);
    servo->start();

    int64_t t;
    double theta, dtheta;
    auto status = servo->get_state(t, theta, dtheta, true);
    if (failed(status))
        throw_runtime_error("received corrupted packed");
    dbg_msg("t=", t, "; theta=", theta, "; dtheta=", dtheta, ";");
    servo->stop();
    return 0;
}

int main(int argc, char const*argv[])
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
        status = test_1(cfg);
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

