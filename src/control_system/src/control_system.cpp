#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/signals.h>
#include <bspline/splines.h>
#include <utils/eigen.h>
#include <butterfly_robot/butterfly.h>
#include <control_system/sliding_controller.h>


using namespace std;


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


    auto f = [&ctrl](BflySignals& signals) {
        if (signals.t < 0.1)
            return true;
        if (!signals.ball_found)
            return false;
        double u = ctrl.process(signals.theta, signals.phi, signals.dtheta, signals.dphi);
        signals.torque = clamp(u, -0.1, 0.1);
        return true;
    };

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
