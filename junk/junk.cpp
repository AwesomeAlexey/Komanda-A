
/**
 * @brief Luenberger observer of the system
 *  theta'' = J u
 * with theta measured
 */
class SpeedObserver
{
private:
    Vec2 x_;
    Vec2 L;
    Mat22 A;
    RVec2 C;
    Vec2 B;
    Mat22 K;
    int64_t t_prev;
    bool first;

public:
    SpeedObserver(double J)
    {
        x_ = {0, 0};
        A = makemat<2,2>(
            0, 1,
            0, 0
        );
        B = makevec(
            0,
            1. / J
        );
        C = makervec(1, 0);
        // L = 0.01 * C.t();
        L = makevec(10, 200);
        K = A - L * C;
        first = true;
    }

    void update(int64_t t_usec, double theta, double u)
    {
        if (first)
        {
            first = false;
            x_ = {theta, 0};
            t_prev = t_usec;
        }

        double y = theta;
        double y_ = (C * x_)(0,0);
        Vec2 dx_ = A * x_  + B * u + L * (y - y_);
        double dt = usec_to_sec(t_usec - t_prev);
        t_prev = t_usec;
        x_ = x_ + dx_ * dt;
    }

    inline double value() const
    {
        return x_(1);
    }
};


};

class Integrator
{
private:
    double x_prev;
    int64_t t_prev;
    double sum;

public:
    Integrator()
    {
        t_prev = -1;
        x_prev = 0.0;
        sum = 0.0;
    }

    void update(int64_t const& t, double const& x)
    {
        if (t_prev < 0)
        {
            t_prev = t;
            x_prev = x;
            sum = 0.0;
        }

        sum += (t - t_prev) * 1e-6 * (x + x_prev) / 2;
        t_prev = t;
        x_prev = x;
    }

    double value() const
    {
        return sum;
    }
};


/**
 * @brief 
 */
class ContinueAngle
{
private:
    double x0;
    int n_revs;

public:
    ContinueAngle()
    {
        n_revs = 0;
        x0 = 0;
    }

    inline void reset()
    {
        n_revs = 0;
        x0 = 0;
    }

    inline double process(double const& x)
    {
        bool const b1 = (x - x0) < - M_PI;
        bool const b2 = (x - x0) > M_PI;

        x0 = x;
        n_revs += b1;
        n_revs -= b2;

        return x + n_revs * 2 * M_PI;
    }
};

/**
 * @brief delay signal
 */

class DelayFilt
{
private:
    struct Signal {
        int64_t t;
        double x;
        Signal() : t(0), x(0.0) {}
    };

    std::vector<Signal> _arr;
    int64_t             _delay;
    int                 _pointer;

public:
    DelayFilt()
    {
        init(0, 0);
    }

    void init(int64_t const& delay_usec, int buf_size = 32)
    {
        _delay = delay_usec;
        _pointer = 0;
        _arr.resize(buf_size);
    }

    DelayFilt(int64_t const& delay_usec, int buf_size = 32)
    {
        init(delay_usec, buf_size);
    }

    inline double process(int64_t const& t_usec, double const& x)
    {
        int last = modulo(_pointer + 1, _arr.size());
        int delayed = last;

        for (int i = _pointer; i != last; i = modulo(i - 1, _arr.size()))
        {
            if (_arr[i].t + _delay < t_usec)
            {
                delayed = i;
                break;
            }
        }

        double res = _arr[delayed].x;

        _pointer = modulo(_pointer + 1, _arr.size());
        _arr[_pointer].t = t_usec;
        _arr[_pointer].x = x;

        return res;
    }
};
