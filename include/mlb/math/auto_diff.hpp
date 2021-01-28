#pragma once

#include <cmath>
#include <iostream>

namespace mlb {
namespace math {

template <class T>
class AD {
 public:
  // operator overload
  // add
  AD& operator+=(const T& s) {
    this->y_ += s;
    return *this;
  }
  AD& operator+=(const AD& other) {
    this->y_ += other.y_;
    this->dy_ += other.dy_;
    return *this;
  }
  // sub
  AD& operator-=(const T& s) {
    this->y_ -= s;
    return *this;
  }
  AD& operator-=(const AD& other) {
    this->y_ -= other.y_;
    this->dy_ -= other.dy_;
    return *this;
  }
  // mul
  AD& operator*=(const T& s) {
    this->y_ *= s;
    this->dy_ *= s;
    return *this;
  }
  AD& operator*=(const AD& other) {
    (*this) = (*this) * other;
    return (*this);
  }
  // div
  AD& operator/=(const T& s) {
    this->y_ /= s;
    this->dy_ /= s;
    return *this;
  }
  AD& operator/=(const AD& other) {
    (*this) = (*this) / other;
    return (*this);
  }

  // friend operator overload
  // plus
  friend AD<T> operator+(const AD<T>& ad) { return {ad.y_, ad.dy_}; }
  // minus
  friend AD<T> operator-(const AD<T>& ad) { return {-ad.y_, -ad.dy_}; }

  // add
  template <class U, class V>
  friend auto operator+(const AD<U>& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ + rhs.y_)>> {
    return {lhs.y_ + rhs.y_, lhs.dy_ + rhs.dy_};
  }
  template <class U, class V>
  friend auto operator+(const AD<U>& lhs, const V& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ + rhs)>> {
    return {lhs.y_ + rhs, lhs.dy_};
  }
  template <class U, class V>
  friend auto operator+(const U& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs + rhs.y_)>> {
    return {lhs + rhs.y_, rhs.dy_};
  }

  // sub
  template <class U, class V>
  friend auto operator-(const AD<U>& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ - rhs.y_)>> {
    return {lhs.y_ - rhs.y_, lhs.dy_ - rhs.dy_};
  }
  template <class U, class V>
  friend auto operator-(const AD<U>& lhs, const V& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ - rhs)>> {
    return {lhs.y_ - rhs, lhs.dy_};
  }
  template <class U, class V>
  friend auto operator-(const U& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs - rhs.y_)>> {
    return {lhs - rhs.y_, rhs.dy_};
  }

  // mul
  template <class U, class V>
  friend auto operator*(const AD<U>& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ * rhs.y_)>> {
    return {lhs.y_ * rhs.y_, lhs.dy_ * rhs.y_ + lhs.y_ * rhs.dy_};
  }
  template <class U, class V>
  friend auto operator*(const AD<U>& lhs, const V& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ * rhs)>> {
    return {lhs.y_ * rhs, lhs.dy_ * rhs};
  }
  template <class U, class V>
  friend auto operator*(const U& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs * rhs.y_)>> {
    return {lhs * rhs.y_, lhs * rhs.dy_};
  }

  // div
  template <class U, class V>
  friend auto operator/(const AD<U>& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ / rhs.y_)>> {
    return {lhs.y_ / rhs.y_, (lhs.dy_ - (lhs.y_ / rhs.y_) * rhs.dy_) / rhs.y_};
  }
  template <class U, class V>
  friend auto operator/(const AD<U>& lhs, const V& rhs)
      -> AD<std::decay_t<decltype(lhs.y_ / rhs)>> {
    return {lhs.y_ / rhs, lhs.dy_ / rhs};
  }
  template <class U, class V>
  friend auto operator/(const U& lhs, const AD<V>& rhs)
      -> AD<std::decay_t<decltype(lhs / rhs.y_)>> {
    return {lhs / rhs.y_, lhs / rhs.dy_};
  }

  // atan
  friend AD<T> atan(const AD<T>& ad) {
    return {std::atan(ad.y_), 1. / (1. + (ad.y_ * ad.y_))};
  }

  friend std::ostream& operator<<(std::ostream& out, const AD<T>& ad) {
    return out << "(y, dy) = (" << ad.GetY() << ", " << ad.GetDy() << ")";
  }

  // less than
  template <class U, class V>
  friend bool operator<(const AD<U>& lhs, const AD<V>& rhs) {
    return lhs.y_ < rhs.y_;
  }
  template <class U, class V>
  friend bool operator<(const U& lhs, const AD<V>& rhs) {
    return lhs < rhs.y_;
  }
  template <class U, class V>
  friend bool operator<(const AD<U>& lhs, const V& rhs) {
    return lhs.y_ < rhs;
  }
  // less than or equal to
  template <class U, class V>
  friend bool operator<=(const AD<U>& lhs, const AD<V>& rhs) {
    return lhs.y_ <= rhs.y_;
  }
  template <class U, class V>
  friend bool operator<=(const U& lhs, const AD<V>& rhs) {
    return lhs <= rhs.y_;
  }
  template <class U, class V>
  friend bool operator<=(const AD<U>& lhs, const V& rhs) {
    return lhs.y_ <= rhs;
  }

  // greater than
  template <class U, class V>
  friend bool operator>(const AD<U>& lhs, const AD<V>& rhs) {
    return lhs.y_ < rhs.y_;
  }
  template <class U, class V>
  friend bool operator>(const U& lhs, const AD<V>& rhs) {
    return lhs < rhs.y_;
  }
  template <class U, class V>
  friend bool operator>(const AD<U>& lhs, const V& rhs) {
    return lhs.y_ < rhs;
  }
  // greater than or equal to
  template <class U, class V>
  friend bool operator>=(const AD<U>& lhs, const AD<V>& rhs) {
    return lhs.y_ <= rhs.y_;
  }
  template <class U, class V>
  friend bool operator>=(const U& lhs, const AD<V>& rhs) {
    return lhs <= rhs.y_;
  }
  template <class U, class V>
  friend bool operator>=(const AD<U>& lhs, const V& rhs) {
    return lhs.y_ <= rhs;
  }

  // constructer
  AD(const T& y = 0., const T& dy = 0.) : y_{y}, dy_{dy} {}

  template <class U>
  explicit AD(const AD<U>& other)
      : y_{static_cast<const T&>(other.y_)},
        dy_{static_cast<const T&>(other.y_)} {}

  AD& operator=(const AD&) = default;
  AD& operator=(AD&&) = default;
  AD(const AD&) = default;
  AD(AD&&) = default;
  AD() = default;

  // setter
  void SetVariable() { dy_ = 1; }
  void SetConstant() { dy_ = 0; }
  // getter
  T GetY() const { return y_; }
  T GetDy() const { return dy_; }

 private:
  T y_;
  T dy_;
};

}  // namespace math

}  // namespace mlb
