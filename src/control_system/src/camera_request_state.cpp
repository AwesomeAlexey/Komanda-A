#include <cppmisc/argparse.h>
#include <cppmisc/traces.h>
#include <cppmisc/timing.h>
#include <cppmisc/threads.h>
#include <butterfly_robot/camera.h>


int test(Json::Value const& jscfg)
{
    int64_t t;
    double x, y;

    std::shared_ptr<Camera> camera = Camera::capture_instance();
    camera->init(jscfg);
    camera->start(true);
    for (int i = 0; i < 10; ++i)
    {
        auto status = camera->get(t, x, y);
        if (failed(status))
        {
            err_msg("can't receive data from cam");
        }

        if (status == Camera::Status::Succeeded)
        {
            info_msg("camera state: ", t, " ", x, " ", y);
        }
        
        sleep_usec(6e+3);
    }
    camera->stop();
    return 1;
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
        Json::Value const& jsconfig = json_load(m["config"]);
        status = test(jsconfig);
    }
    catch (std::exception const& e)
    {
        err_msg("failed: ", e.what());
        status = -1;
    }
    catch (...)
    {
        err_msg("failed: undef");
        status = -1;
    }

    return status;
}
