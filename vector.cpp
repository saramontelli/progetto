#include "vector.hpp"

#include <cmath>

namespace math {

Vector::Vector() : x_{0.0f}, y_{0.0f} {};

Vector::Vector(float x, float y) : x_{x}, y_{y} {};

float Vector::get_x() const { return x_; }

float Vector::get_y() const { return y_; }

void Vector::set_x(float newX) { x_ = newX; }

void Vector::set_y(float newY) { y_ = newY; }

Vector Vector::operator+(const Vector& w) const {
  Vector result = *this;
  return result += w;
};

Vector Vector::operator-(const Vector& w) const {
  Vector neg_w = {-w.x_, -w.y_};
  return *this + neg_w;
}

Vector Vector::operator*(float a) const {
  Vector product{x_ * a, y_ * a};
  return product;
}

Vector& Vector::operator+=(const Vector& w) {
  x_ += w.x_;
  y_ += w.y_;
  return *this;
}

bool Vector::operator==(const Vector& w) const {
  return (x_ == w.x_ && y_ == w.y_);
}

float Vector::dot(const Vector& w) const {
  float dot = x_ * w.x_ + y_ * w.y_;
  return dot;
}

float Vector::norm() const { return std::sqrt(x_ * x_ + y_ * y_); }

Vector Vector::shortest_delta(const Vector& w, float x_max, float y_max) const {
  float dx = w.x_ - x_;
  float dy = w.y_ - y_;

  if (dx > x_max / 2.0f)
    dx -= x_max;
  else if (dx < -x_max / 2.0f)
    dx += x_max;

  if (dy > y_max / 2.0f)
    dy -= y_max;
  else if (dy < -y_max / 2.0f)
    dy += y_max;

  return Vector(dx, dy);
}

float Vector::distance(const Vector& w, float x_max, float y_max) const {
  return shortest_delta(w, x_max, y_max).norm();
}
}  // namespace math