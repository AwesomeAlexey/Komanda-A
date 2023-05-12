#pragma once

class Integrator
{
public:
	Integrator()
	{
		_t_prev = -1;
		_u_prev = 0;
		_sum = 0;
	}

	double process(double t, double u)
	{
		if (_t_prev < 0)
		{
			_t_prev = t;
			_u_prev = u;
		}

		_sum += (u + _u_prev) * (t - _t_prev) / 2;
		_t_prev = t;
		_u_prev = u;
		return _sum;
	}

	inline double operator () (double t, double u)
	{
		return process(t, u);
	}

private:
	double _t_prev, _u_prev, _sum;
};
