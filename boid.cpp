#include "boid.hpp"

#include <cmath>

namespace math {
// costruttore di default
Boid::Boid() : pos_(0., 0.), vel_(0., 0.), predator_(false) {}
// costruttore con parametri
Boid::Boid(Vector pos, Vector vel, bool predator)
    : pos_(pos), vel_(vel), predator_(predator) {}

void Boid::set_predator(bool value) { predator_ = value; }

Vector Boid::get_pos() const { return pos_; }
Vector Boid::get_vel() const { return vel_; }
bool Boid::get_predator() const { return predator_; }

bool Boid::operator==(const Boid& boid) const { return pos_ == boid.get_pos(); }

std::vector<Boid> Boid::get_neighbors(const std::vector<Boid>& all_boids,
                                      float d, float x_max, float y_max) const {
  std::vector<Boid> near;
  for (const auto& boid : all_boids) {
    if (pos_ == boid.get_pos()) {
      continue;
    } else if (pos_.distance(boid.get_pos(), x_max, y_max) < d) {
      near.push_back(boid);
    }
  }
  return near;
}
Vector Boid::separation(const std::vector<Boid>& near, float s, float d_s,
                        float x_max, float y_max) const {
  Vector v_1(0., 0.);

  for (const auto& boid : near) {
    float dist = pos_.distance(boid.get_pos(), x_max, y_max);

    if (dist < d_s && dist > 0.001f) {
      float dx = boid.get_pos().get_x() - pos_.get_x();
      float dy = boid.get_pos().get_y() - pos_.get_y();
      if (dx > x_max / 2)
        dx -= x_max;
      else if (dx < -x_max / 2)
        dx += x_max;

      if (dx > x_max / 2)
        dx -= x_max;
      else if (dx < -x_max / 2)
        dx += x_max;

      if (dy > y_max / 2)
        dy -= y_max;
      else if (dy < -y_max / 2)
        dy += y_max;

        v_1 += Vector(dx, dy);
      }
    }
    return v_1 * (-s);
  }

  Vector Boid::alignment(const std::vector<Boid>& near, float a) const {
    if (near.empty()) return Vector(0.f, 0.f);  // evita che si divida per zero
    Vector v_2{0.f, 0.f};
    for (const auto& boid : near) {
      v_2 += boid.get_vel();
    }
    v_2 =
        v_2 * (1.0f / static_cast<float>(near.size()));  // media delle velcità
    return (v_2 - this->get_vel()) * a;
  }

  Vector Boid::cohesion(const std::vector<Boid>& near, float c, float x_max,
                        float y_max) const {
    if (near.empty()) return Vector(0.f, 0.f);

    Vector relative_sum{0.f, 0.f};

    for (const auto& boid : near) {
      // Calcoliamo lo spostamento relativo tra il boid e il suo vicino
      float dx = boid.get_pos().get_x() - pos_.get_x();
      float dy = boid.get_pos().get_y() - pos_.get_y();

      // Correzione toroidale: se la distanza è > metà mappa,
      // il percorso più breve è dall'altra parte
      if (dx > x_max / 2)
        dx -= x_max;
      else if (dx < -x_max / 2)
        dx += x_max;

      if (dy > y_max / 2)
        dy -= y_max;
      else if (dy < -y_max / 2)
        dy += y_max;

      relative_sum += Vector(dx, dy);
    }

    // La media degli spostamenti relativi punta verso il baricentro toroidale
    Vector avg_relative_pos =
        relative_sum * (1.f / static_cast<float>(near.size()));

    return avg_relative_pos * c;
  }

  void Boid::speed_limit(float vel_max, float vel_min) {
    float speed = this->get_vel().norm();
    if (speed > vel_max) {
      vel_ = vel_ * (vel_max / speed);
    };
    if (speed > 0.f && speed < vel_min) {
      vel_ = vel_ * (vel_min / speed);
    }
  }

  void Boid::wrap_position(float x_max, float y_max) {
    float x = pos_.get_x();
    float y = pos_.get_y();

    // Controllo asse X
    if (x <= 0.f) {
      pos_.set_x(x_max);
    } else if (x >= x_max) {
      pos_.set_x(0.);
    }

    // Controllo asse Y
    if (y <= 0.f) {
      pos_.set_y(y_max);
    } else if (y >= y_max) {
      pos_.set_y(0.);
    }
  }

  void Boid::change_pos(const Vector& d_x) { pos_ += d_x; }

  void Boid::change_vel(const Vector& d_v) { vel_ += d_v; }
}  // namespace math