#pragma once

#include <memory>
#include <networking/tcp.h>
#include <cppmisc/json.h>
#include <cppmisc/timing.h>
#include "device_manager.h"
#include "servo_protocol.h"


class Servo
{
private:
    std::shared_ptr<Connection> m_connection;
    std::string     m_server_ip;
    int             m_server_port;

    Servo();
    Servo(Servo const&) = delete;

public:
    void init(Json::Value const& jscfg);
    void start();
    void stop();

    enum class Status {
        ConnectionClosed = -1000,
        NotConnected,
        SendFailed,
        NoData = 0,
        Succeeded
    };

    Status get_state(int64_t& t, double& theta, double& dtheta, bool blocking);
    Status set_torque(double const& torque);
    static std::shared_ptr<Servo> capture_instance();
};

inline bool failed(Servo::Status st)
{
    return static_cast<int>(st) < 0;
}
