#include <cppmisc/traces.h>
#include <cppmisc/threads.h>
#include <cppmisc/argparse.h>
#include <cppmisc/signals.h>
#include <bspline/splines.h>
#include <utils/eigen.h>
#include <utils/csv_logger.h>
#include <butterfly_robot/butterfly.h>
#include <control_system/sliding_controller.h>


using namespace std;

std::unique_ptr<CSVLogger> make_logger(Json::Value const& jscfg, std::string const& format)
{
    auto loggercfg = json_get(jscfg, "logger");
    auto path = json_get<std::string>(loggercfg, "saveto");
    return std::make_unique<CSVLogger>(path, format);
}

int launch(Json::Value const& jscfg, Json::Value const& ctrlcfg)
{
    Butterfly bfly;
    bfly.init(jscfg);
    bool stop = false;
    auto stop_handler = [&stop, &bfly]() { stop = true; bfly.stop(); };
    SysSignals::instance().set_sigint_handler(stop_handler);
    SysSignals::instance().set_sigterm_handler(stop_handler);
    SlidingController ctrl;
    ctrl.load(ctrlcfg);
    auto logger = make_logger(jscfg, 
        "t=%2.5f,x=%2.5f,y=%2.5f,theta=%2.5f,dtheta=%2.5f,phi=%2.5f,dphi=%2.5f,u=%2.5f");

    auto f = [&ctrl,&logger](BflySignals& signals) {
        if (signals.t < 0.1)
            return true;
        if (!signals.ball_found)
            return false;
        double u = ctrl.process(signals.t, signals.theta, signals.phi, signals.dtheta, signals.dphi);
        signals.torque = clamp(u, -0.1, 0.1);
        logger->write(
            signals.t, signals.x, signals.y, signals.theta, signals.dtheta, 
            signals.phi, signals.dphi, signals.torque);
        return true;
    };

    set_thread_rt_priotiy(-1, 50);
    bfly.start(f);
    return 0;
}

int main(int argc, char const* argv[])
{
    Arguments args({
        Argument("-c", "config", "path to the butterfly robot config file", "", ArgumentsCount::One),
        Argument("-f", "controller", "path to controller config file", "", ArgumentsCount::One)
    });

    int status = 0;

    try
    {
        auto&& m = args.parse(argc, argv);
        Json::Value const& butcfg = json_load(m["config"]);
        traces::init(json_get(butcfg, "traces"));
        Json::Value const& ctrlcfg = json_load(m["controller"]);
        launch(butcfg, ctrlcfg);
    }
    catch (exception const& e)
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
