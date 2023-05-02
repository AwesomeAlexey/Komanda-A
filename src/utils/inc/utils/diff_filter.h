#pragma once

#include "eigen.h"

/**
 * @brief state space model coefficients
 */
struct StateSpace {
  Eigen::MatrixXd A, B, C, D;
};

/**
 * @brief transfer function fraction coefficients
 */
struct TransferFunction {
  Eigen::VectorXd num, denom;
};

/**
 * @brief The transfer function of the differentiating filter of the form
 *  @f$ y = \frac{s^k}{(Ts+1)^d} u @f$
 * @param order order of the filter
 * @param period time constant of the filter
 */
TransferFunction diff_filter_tf(int order, double period);

/**
 * @brief The state space representation of the differentiating filter of the form
 *  @f$ y = \frac{s^k}{(Ts+1)^d} u @f$
 * @param order order of the filter
 * @param period time constant of the filter
 */
StateSpace diff_filter_ss(int order, double period);

/**
 * @brief Differentiating filter of the form
 *  @f$ y = \frac{s^k}{(Ts+1)^d} u @f$
 * where d is the degree of the filter
 */
template <int Deg>
class DiffFilter {

public:
  /**
   * @param[in] period is the filter time constant T, in src
   */
  DiffFilter(double period)
  {
    auto ss = diff_filter_ss(Deg, period);
    A_ = ss.A;
    Ainv_ = A_.inverse();
    B_ = ss.B;
    C_ = ss.C;
    D_ = ss.D;
    t_ = -1.0;
  }

  /**
   * @brief The function updates the filter's internal state and returns
   *  values of computed derivatives
   * @param u input signal
   * @param t input signal measurement time in seconds
   * @return filter output y
   */
  inline Eigen::Matrix<double, Deg + 1, 1> update(double t, double u)
  {
    integrate(t, u);
    y_ = C_ * x_ + D_ * u;
    return y_;
  }

  /**
   * @brief get filter's output
   */
  inline double value(int nderiv = 0) const { return y_(nderiv); }

private:
  Eigen::Matrix<double, Deg, Deg> A_, Ainv_;
  Eigen::Matrix<double, Deg, 1> B_;
  Eigen::Matrix<double, Deg + 1, Deg> C_;
  Eigen::Matrix<double, Deg + 1, 1> D_;
  Eigen::Matrix<double, Deg, 1> x_;
  Eigen::Matrix<double, Deg + 1, 1> y_;
  double t_;

  void integrate(double t, double u)
  {
    if (t_ < 0) {
      x_.fill(0.);
      x_(0) = u;
      t_ = t;
    }

    double dt = t - t_;
    t_ = t;
    Eigen::Matrix<double, Deg, Deg> expAdt = (A_ * dt).exp();
    auto I = Eigen::Matrix<double, Deg, Deg>::Identity();
    x_ = expAdt * x_ + Ainv_ * (expAdt - I) * B_ * u;
  }

};

class BoundedDerivativeFilter {
public:
  /**
   * @brief Second order diff filter with output signal bounded derivative
   * @param maxriseup signal maximum rise up derivative value
   * @param maxriseup signal maximum fall down derivative value
   * @param period filter time constant
   */
  BoundedDerivativeFilter(double maxriseup, double maxfalldown, double period);
  std::tuple<double,double> update(double t, double u);
  double value(int nderiv = 0) const;

private:
  const double riseup_max_;
  const double falldown_max_;
  Mat22 A_, Ainv_;
  Vec2 B_;
  Vec2 x_;
  double t_;
};
