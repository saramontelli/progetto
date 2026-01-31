#include "vector.hpp"

#include <cmath>

namespace math {
// implementiamo la classe vector, cioè come si construisce e come si usa un
// oggetto della classe Vector serve: un costruttore di default, un costruttore
// parametrico, getter e setter

// costruttore: stesso nome della classe, nessun tipo(neanche void)

Vector::Vector() : x_{0.0f}, y_{0.0f} {};  // costruttore di default

// costruttore parametrico: NomeClasse(Tipo parametro1, Tipo parametro2) { ... }

Vector::Vector(float x, float y) : x_{x}, y_{y} {};

float Vector::get_x() const { return x_; }

float Vector::get_y() const { return y_; }

void Vector::set_x(float newX) { x_ = newX; }

void Vector::set_y(float newY) { y_ = newY; }
// sintassi per l'implementazione di una funzione membro di una classe fuori
// dall'header: TipoDiRitorno NomeClasse::NomeFunzione(Parametri)
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

float Vector::distance(const Vector& w, float x_max, float y_max) const {
  float dx = std::abs(x_ - w.x_);
  float dy = std::abs(y_ - w.y_);

  if (dx > x_max / 2.0f) dx = x_max - dx;
  if (dy > y_max / 2.0f) dy = y_max - dy;

  return std::sqrt(dx * dx + dy * dy);
}
}  // namespace math