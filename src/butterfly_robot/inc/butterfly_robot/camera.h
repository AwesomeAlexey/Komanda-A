#pragma once

#include <string>
#include <memory>
#include <networking/tcp.h>
#include <cppmisc/json.h>
#include <cppmisc/serializer.h>


class Camera
{
private:
    ser::PacketReaderPtr _con_reader;
    std::string _host;
    int _port;

    Camera();
    Camera(Camera const&) = delete;

public:
    ~Camera();
    void init(Json::Value const& jscfg);

    enum class Status {
        Failed = -1000, 
        NoData = 0,
        Succeeded
    };
    Status get(int64_t& ts_usec, double& x, double& y);

    void start(bool blocking_read=false);
    void stop();

    static std::shared_ptr<Camera> capture_instance();
};

inline bool failed(Camera::Status st)
{
    return static_cast<int>(st) < 0;
}
