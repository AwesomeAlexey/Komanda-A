#include <butterfly_robot/servo.h>


Servo::Servo() {}

void Servo::init(Json::Value const& jscfg)
{
    auto const& servocfg = json_get<Json::Value>(jscfg, "servo");
    json_get(servocfg, "ip", m_server_ip);
    json_get(servocfg, "port", m_server_port);
}

void Servo::start()
{
    m_connection = Connection::connect(m_server_ip, m_server_port);
    if (!m_connection)
        throw_runtime_error("can't connect to ", m_server_ip, ":", m_server_port);

    int64_t t = epoch_usec();
    servocmd::CmdPack pack;
    servocmd::init_cmd_start(t, pack);
    if (!m_connection->write(reinterpret_cast<char*>(&pack), sizeof(pack)))
        throw_runtime_error("servo connection broken");
}

void Servo::stop()
{
    m_connection.reset();
}

Servo::Status Servo::get_state(int64_t& t, double& theta, double& dtheta, bool blocking)
{
    if (!m_connection)
        throw_runtime_error("can't read state: not connected");

    servocmd::InfoPack ans;
    int status = m_connection->read(reinterpret_cast<char*>(&ans), sizeof(ans), blocking);
    if (status == 0)
    {
        if (blocking)
        {
            dbg_msg("connection was closed by server");
            return Status::ConnectionClosed;
        }
        return Status::NoData;
    }

    if (status < 0)
        throw_runtime_error("can't read from server");

    if (status == sizeof(ans))
    {
        if (!servocmd::verify_pack(ans))
            throw_runtime_error("server sent corrupted answer");

        t = ans.t;
        theta = ans.theta;
        dtheta = ans.dtheta;
        return Status::Succeeded;
    }

    throw_runtime_error("unexpected answer from server");
}

Servo::Status Servo::set_torque(double const& torque)
{
    if (!m_connection)
    {
        err_msg("can't set torque: not connected");
        return Status::NotConnected;
    }

    int64_t t = epoch_usec();
    servocmd::CmdPack pack;
    servocmd::init_cmd_torque(t, torque, pack);
    if (!m_connection->write(reinterpret_cast<char*>(&pack), sizeof(pack)))
    {
        err_msg("can't send packet to server");
        return Status::NoData;
    }

    return Status::Succeeded;
}

std::shared_ptr<Servo> Servo::capture_instance()
{
    auto& devices = Devices::get_instance();
    auto instance = devices.capture_instance<Servo>();
    if (instance == nullptr)
    {
        instance.reset(new Servo());
        devices.append(instance);
    }

    return instance;
}
