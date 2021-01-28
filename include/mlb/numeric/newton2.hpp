// CopyRight
#pragma once

#include <array>
#include <cmath>
#include <iostream>

#include "mlb/math/auto_diff.hpp"
#include "mlb/math/matrix2.hpp"

namespace mlb {
namespace numeric {

template <class T>
class Newton2 {
 public:
  explicit Newton2(const mlb::math::Vector2<T>&);
  void SetX();
  void SetYDy();
  void UpdateXk(const mlb::math::Matrix2<T>&);
  bool IsEnd() {
    return std::hypot(prev_x_(0) - xk_(0), prev_x_(1) - xk_(1)) < EPSILON;
  }
  void Calc();

 private:
  static constexpr std::size_t MAX_ITERATIONS = 10000;
  static constexpr double EPSILON = 1e-10;
  static constexpr std::size_t N = 2;
  mlb::math::Vector2<T> xk_;
  mlb::math::Vector2<T> prev_x_;
  mlb::math::Vector2<T> f_;
  mlb::math::Vector2<mlb::math::AD<T>> x_;
  mlb::math::Matrix2<T> jacobian_;
};

template <class T>
Newton2<T>::Newton2(const mlb::math::Vector2<T>& xk) : xk_(xk) {}

template <class T>
void Newton2<T>::SetX() {
  for (size_t i = 0; i < N; i++) {
    x_(i) = mlb::math::AD<T>{xk_(i)};
  }
}

template <class T>
void Newton2<T>::SetYDy() {
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      x_(j).SetVariable();
      // problem1
      mlb::math::AD<T> y =
          (i == 0 ? x_(0) + x_(1) : x_(0) * x_(0) + x_(1) * x_(1) - 5);

      // problem2
      // mlb::math::AD<T> y =
      //     (i == 0 ? x_(1) * x_(1) * x_(1) - 3 * x_(0) * x_(0) * x_(1)
      //             : x_(0) * x_(0) * x_(0) - 3 * x_(0) * x_(1) * x_(1) - 5);

      jacobian_(i, j) = y.GetDy();
      f_(i) = y.GetY();
      x_(j).SetConstant();
    }
  }
}

template <class T>
void Newton2<T>::UpdateXk(const mlb::math::Matrix2<T>& inverse) {
  for (size_t i = 0; i < N; i++) {
    prev_x_(i) = xk_(i);
  }
  xk_ -= inverse * f_;
}

template <class T>
void Newton2<T>::Calc() {
  for (size_t i = 0; i < MAX_ITERATIONS; i++) {
    SetX();
    SetYDy();
    UpdateXk(jacobian_.Inverse());

    if (IsEnd()) {
      std::printf("%ld %f %f\n", i, xk_(0), xk_(1));
      break;
    }
    if (i == MAX_ITERATIONS - 1) {
      std::printf("%ld\n", i);
    }
  }
}

}  // namespace numeric

}  // namespace mlb
