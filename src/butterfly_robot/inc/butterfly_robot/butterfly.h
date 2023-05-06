#pragma once

#include <memory>
#include <stdexcept>
#include <cppmisc/json.h>
#include <utils/filters.h>
#include <utils/delay_filter.h>
#include <utils/diff_filter.h>
#include "servo.h"
#include "camera.h"


struct BflySignals
{
    bool ball_found;
    double t;
    double theta;
    double dtheta;
    double phi;
    double dphi;
    double x;
    double vx;
    double y;
    double vy;
    double torque;
};

class Butterfly
{
private:
    std::shared_ptr<Servo> m_servo;
    std::shared_ptr<Camera> m_camera;

    EulerDiff   m_diff_x;
    EulerDiff   m_diff_y;
    DelayFilt   m_delay_theta;
    DelayFilt   m_delay_dtheta;

    int64_t     m_sync_delay;
    double      m_theta, m_dtheta;
    double      m_x, m_y;
    double      m_vx, m_vy;
    double      m_phi, m_dphi;
    bool        m_stop;
    bool        m_ball_found;

    void measure();
    void get_signals(int64_t const& t, BflySignals& signals);

public:
    using callback_t = std::function<bool(BflySignals&)>;

    Butterfly();
    ~Butterfly();

    void init(Json::Value const& jscfg);
    void stop();
    void start(callback_t const& cb);    
};
