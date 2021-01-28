// CopyRight
#pragma once

#include <array>
#include <cmath>

#include "mlb/math/vector2.hpp"

namespace mlb {
namespace math {

template <class T>
class Matrix2 {
 public:
  Matrix2<T> Inverse();
  T Det();
  const T& operator()(std::size_t i, std::size_t j) const noexcept {
    return matrix_[i][j];
  }
  T& operator()(std::size_t i, std::size_t j) noexcept { return matrix_[i][j]; }
  template <class U, class V>
  friend auto operator*(const Matrix2<U>& lhs,
                        const mlb::math::Vector2<V>& rhs) {
    mlb::math::Vector2<T> res;
    for (size_t i = 0; i < 2; i++) {
      res(i) = lhs(i, 0) * rhs(0) + lhs(i, 1) * rhs(1);
    }
    return res;
  }

 private:
  static constexpr std::size_t N = 2;
  std::array<std::array<T, N>, N> matrix_;
};

template <class T>
T Matrix2<T>::Det() {
  return matrix_[0][0] * matrix_[1][1] - matrix_[0][1] * matrix_[1][0];
}

template <class T>
Matrix2<T> Matrix2<T>::Inverse() {
  const T det = this->Det();
  Matrix2<T> inverse;
  for (std::size_t i = 0; i < N; i++) {
    for (std::size_t j = 0; j < N; j++) {
      inverse(i, j) = std::pow(-1, i ^ j) * matrix_[1 - j][1 - i] / det;
    }
  }
  return inverse;
}

}  // namespace math
}  // namespace mlb
