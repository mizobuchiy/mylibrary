// CopyRight
#pragma once

#include <array>

namespace mlb {
namespace math {
template <class T>
class Vector2 {
 public:
  const T& operator()(std::size_t i) const noexcept { return vector_[i]; }
  T& operator()(std::size_t i) noexcept { return vector_[i]; }
  Vector2<T>& operator=(const Vector2<T>& rhs) noexcept {
    for (size_t i = 0; i < N; i++) {
      vector_[i] = rhs(i);
    }
    return *this;
  }
  Vector2<T>& operator-=(const Vector2<T>& rhs) noexcept {
    for (size_t i = 0; i < N; i++) {
      vector_[i] -= rhs(i);
    }
    return *this;
  }

 private:
  static constexpr std::size_t N = 2;
  std::array<T, N> vector_;
};
}  // namespace math
}  // namespace mlb
