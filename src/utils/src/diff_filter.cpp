#include <assert.h>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>
#include <utils/diff_filter.h>
#include <utils/utils.h>


using namespace Eigen;


static double binom(int n, int k)
{
  double b = 1;
  if (k < n - k) {
    for (int i = 1; i <= k; ++i)
      b *= double(n - k + i) / i;
  } else {
    for (int i = 1; i <= n - k; ++i)
      b *= double(k + i) / i;
  }
  return b;
}

static int binomi(int n, int k)
{
  int num = 1;
  int den = 1;
  if (k < n - k) {
    for (int i = 1; i <= k; ++i) {
      num *= n - k + i;
      den *= i;
    }
  } else {
    for (int i = 1; i <= n - k; ++i) {
      num *= k + i;
      den *= i;
    }
  }
  return num / den;
}

TransferFunction diff_filter_tf(int order, double period)
{
  VectorXd coefs;
  coefs.resize(order + 1);
  double period_k = 1;

  for (int k = 0; k <= order; ++k) {
    coefs(k) = binom(order, k) * period_k;
    period_k *= period;
  }
  VectorXd num(1);
  num.fill(1);
  return {num, coefs};
}

StateSpace diff_filter_ss(int order, double period)
{
  auto [num, denom] = diff_filter_tf(order, period);
  double an = denom(order);

  MatrixXd A(order, order);
  A.fill(0);
  A.diagonal(1).fill(1);
  A.block(order - 1, 0, 1, order) = -denom.block(0, 0, order, 1).transpose() / an;

  VectorXd B(order);
  B.fill(0);
  B(order - 1) = 1 / an;

  MatrixXd C = MatrixXd::Identity(order + 1, order);
  C.block(order, 0, 1, order) = -denom.block(0, 0, order, 1).transpose() / an;

  VectorXd D(order + 1);
  D.fill(0);
  D(order) = 1 / an;

  return {A, B, C, D};
}

BoundedDerivativeFilter::BoundedDerivativeFilter(double maxriseup, double maxfalldown, double period) :
  riseup_max_(maxriseup),
  falldown_max_(maxfalldown)
{
  if (maxriseup <= 0)
    throw std::invalid_argument("maxriseup must be greater than zero");

  if (maxfalldown <= 0)
    throw std::invalid_argument("maxfalldown must be greater than zero");

  A_ << 
    0, 1,
    -1/square(period), -2/period;
  B_ << 0, 1/square(period);
  Ainv_ = A_.inverse();
  t_ = -1;
}

std::tuple<double,double> BoundedDerivativeFilter::update(double t, double u)
{
  if (t_ < 0) {
    x_.fill(0);
    x_(0) = u;
    t_ = t;
    return std::make_tuple(x_(0), x_(1));
  }

  double dt = t - t_;
  t_ = t;
  if (dt <= 0) {
    return std::make_tuple(x_(0), x_(1));
  }

  Matrix2d Ad = (A_ * dt).exp();
  Matrix2d I = Matrix2d::Identity();
  Vector2d Bd = Ainv_ * (Ad - I) * B_;

  auto C = makervec(0., 1.);
  double CB = (C * Bd)(0);
  double CAx = (C * Ad * x_)(0);

  double derivative_min, derivative_max;
  if (x_(0) >= 0) {
    derivative_min = -falldown_max_;
    derivative_max = riseup_max_;
  } else {
    derivative_min = -riseup_max_;
    derivative_max = falldown_max_;
  }

  double umin = (derivative_min - CAx) / CB;
  double umax = (derivative_max - CAx) / CB;
  u = std::clamp(u, umin, umax);
  x_ = Ad * x_ + Bd * u;
  return std::make_tuple(x_(0), x_(1));
}

double BoundedDerivativeFilter::value(int nderiv) const
{
  if (nderiv < 0 || nderiv > 2)
    throw std::invalid_argument("I can find up to first derivative only");
  return x_(nderiv);
}
