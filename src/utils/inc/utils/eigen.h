#pragma once

#include <Eigen/Core>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

using RVec2 = Eigen::RowVector2d;
using RVec3 = Eigen::RowVector3d;
using RVec4 = Eigen::RowVector4d;

using Mat21 = Eigen::Matrix<double, 2, 1>;
using Mat22 = Eigen::Matrix<double, 2, 2>;
using Mat12 = Eigen::Matrix<double, 1, 2>;
using Mat34 = Eigen::Matrix<double, 3, 4>;
using Mat43 = Eigen::Matrix<double, 4, 3>;

/**
 * @brief create vector
 */
template <typename First, typename... Tail>
inline Eigen::Matrix<First, sizeof...(Tail) + 1, 1> makevec(First first, Tail... tail)
{
  constexpr int n = sizeof...(Tail) + 1;
  Eigen::Matrix<First, n, 1> v;
  First* pdst = &v(0);
  *(pdst++) = first;
  ((*(pdst++) = tail), ...);
  return v;
}

/**
 * @brief create row vector
 */
template <typename First, typename... Tail>
inline Eigen::Matrix<First, 1, sizeof...(Tail) + 1> makervec(First first, Tail... tail)
{
  constexpr int n = sizeof...(Tail) + 1;
  Eigen::Matrix<First, 1, n> v;
  First* pdst = &v(0);
  *(pdst++) = first;
  ((*(pdst++) = tail), ...);
  return v;
}

/**
 * @brief create mat from the given arguments row-by-row
 *  a1 a2 a3
 *  a4 a5 a6
 *  ...
 */
template <int Rows, int Cols, typename First, typename... Tail>
inline Eigen::Matrix<First, Rows, Cols> makemat(First first, Tail... tail)
{
  constexpr int n = sizeof...(Tail) + 1;
  static_assert(Rows * Cols == n, "Input arguments dont' match the matrix dimension");
  Eigen::Matrix<First, Rows, Cols> m;
  m(0, 0) = first;
  int i = 1;
  ((m(i / Cols, i % Cols) = tail, ++ i), ...);
  return m;
}
