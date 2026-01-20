#include "boid.hpp"

#include <cmath>

namespace math {
// costruttore di default
Boid::Boid() : pos_(0., 0.), vel_(0., 0.) {}
// costruttore con parametri
Boid::Boid(Vector pos, Vector vel) : pos_(pos), vel_(vel) {}

Vector Boid::get_pos() const { return pos_; }
Vector Boid::get_vel() const { return vel_; }

bool Boid::operator==(const Boid& boid) const { return pos_ == boid.get_pos(); }

std::vector<Boid> Boid::get_neighbors(const std::vector<Boid>& all_boids,
                                      float d) const {
  std::vector<Boid> near;
  for (const auto boid : all_boids) {
    if (pos_ == boid.get_pos()) {
      continue;
    } else if (pos_.distance(boid.get_pos()) < d) {
      near.push_back(boid);
    }
  }
  return near;
}
Vector Boid::separation(const std::vector<Boid>& near, float s,
                        float d_s) const {
  Vector v_1(0., 0.);
  for (const auto& boid : near) {
    if (pos_.distance(boid.get_pos()) < d_s) {
      v_1 += (pos_ - boid.get_pos()) * s;
    }
  }
  return v_1;
}

Vector Boid::alignment(const std::vector<Boid>& near, float a) const {
  if (near.empty()) return Vector(0.f, 0.f);  // evita che si divida per zero
  Vector v_2{0.f, 0.f};
  for (auto& boid : near) {
    v_2 += boid.get_vel();
  }
  v_2 = v_2 * (1.0f / near.size());  // media delle velcità
  return (v_2 - this->get_vel()) * a;
}

Vector Boid::cohesion(const std::vector<Boid>& near, float c) const {
  if (near.empty()) return Vector(0.f, 0.f);

  Vector x_c{0.f, 0.f};
  Vector v_3{0.f, 0.f};

  for (auto& boid : near) {
    x_c += boid.get_pos();
  }

  x_c = x_c * (1.f / near.size());
  v_3 = (x_c - this->get_pos()) * c;
  return v_3;
}

void Boid::change_pos(const Vector& d_x) { pos_ += d_x; }

void Boid::change_vel(const Vector& d_v) { vel_ += d_v; }
}  // namespace math
