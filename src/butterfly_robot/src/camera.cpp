#include <string.h>
#include <cppmisc/traces.h>
#include <cppmisc/timing.h>
#include <butterfly_robot/device_manager.h>
#include <butterfly_robot/camera.h>

using namespace std;


Camera::Camera()
{
    _host = "";
    _port = 0;
}

Camera::~Camera()
{
}

void Camera::init(Json::Value const& jscfg)
{
    auto const& jscam = json_get(jscfg, "camera");
    json_get<std::string>(jscam, "ip", _host);
    json_get(jscam, "port", _port);
}

void Camera::start(bool blocking_read)
{
    if (_host.empty())
        throw_runtime_error("camera is not initialized yet; run init(...)");

    auto cnct = Connection::connect(_host, _port);
    _con_reader = ser::make_pack_reader([cnct,blocking_read](char* p, int n) {
        if (!cnct)
            return -1;
        return cnct->read(p, n, blocking_read);
    });
    char buf[1024];
    int len = ser::pack(buf, sizeof(buf), 
        "ts", epoch_usec(), 
        "cmd", "start"
    );
    cnct->write(buf, len);
}

void Camera::stop()
{
    _con_reader = nullptr;
}

Camera::Status Camera::get(int64_t& ts_usec, double& x, double& y)
{
    if (!_con_reader)
        throw_runtime_error("it seems camera is down");

    ser::Packet pack;
    int status = _con_reader->fetch_next(pack);
    if (status < 0)
        throw_runtime_error("camera connection closed, no data");

    if (status == 0)
    {
        dbg_msg("camera no data");
        return Status::NoData;
    }

    bool good;
    status = pack.get("good", good);
    if (status <= 0)
        throw_runtime_error("camera data corrupted");

    if (!good)
    {
        dbg_msg("bad flag received");
        return Status::Failed;
    }

    status = pack.get("x", x, "y", y, "ts", ts_usec);
    if (status <= 0)
        throw_runtime_error("camera data corrupted");

    return Status::Succeeded;
}

shared_ptr<Camera> Camera::capture_instance()
{
    auto& devices = Devices::get_instance();
    auto instance = devices.capture_instance<Camera>();
    if (instance == nullptr)
    {
        instance.reset(new Camera());
        devices.append(instance);
    }

    return instance;
}
