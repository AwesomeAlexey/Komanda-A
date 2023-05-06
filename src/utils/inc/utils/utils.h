#pragma once

#include <cmath>
#include <cstdlib>


inline constexpr double _PI = 3.1415926535897932384626433832795;
inline constexpr double _PI_2 = _PI / 2;
inline constexpr double _2_PI = 2 * _PI;
inline constexpr double _4_PI = 4 * _PI;

template <typename T>
inline bool in_diap(T const& val, T const& minval, T const& maxval)
{
    return val >= minval && val <= maxval;
}

template <typename T>
inline T square(T const& x)
{
	return x * x;
}

template <typename T>
inline T sign(T const& x)
{
	return (T)(x > 0) * (T)2 - (T)1;
}

inline double modulo(double a, double b)
{
	int n = (int)floor(a / b);
	return a - n * b;
}

inline double excess(double a, double b)
{
    long n = lround(a / b);
    return a - n * b;
}

inline double triangle(double t, double T)
{
	return fabs(modulo(t / (T * 0.25), 4.) - 2.) - 1.;
}

inline double random(double a, double b)
{
	double r = std::rand();
	return a + (b - a) * (r / RAND_MAX);
}

inline double normalize_angle(double a, double angle_from=-_PI, double angle_to=_PI)
{
    int n = std::floor((a - angle_from) / (angle_to - angle_from));
    return a - n * (angle_to - angle_from);
}

template <typename First, typename ... Tail>
inline bool all_equal(First first, First second, Tail ... tail)
{
  if constexpr (sizeof...(Tail) == 0)
  {
    return first == second;
  }
  else
  {
    return first == second && all_equal(second, tail...);
  }
}