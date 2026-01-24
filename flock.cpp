#include "flock.hpp"

#include <cmath>

#include "boid.hpp"
#include "vector.hpp"

namespace math {

Flock::Flock(const float d, const float ds, const float s, const float a,
             const float c, const float max_speed, const float min_speed)
    : d_(d),
      d_s_(ds),
      s_(s),
      a_(a),
      c_(c),
      vel_max(max_speed),
      vel_min(min_speed) {};
void Flock::add_boids(const Boid& new_boid) { flock_.push_back(new_boid); }

const std::vector<Boid>& Flock::get_flock() const { return flock_; };

Vector Flock::flock_separation(const Boid& current_boid) const {
  // trovo i vicini
  std::vector<Boid> neighbors = current_boid.get_neighbors(flock_, d_);
  // calcolo la separazione
  return current_boid.separation(neighbors, s_, d_s_);
};

Vector Flock::flock_alignment(const Boid& current_boid) const {
  std::vector<Boid> neighbors = current_boid.get_neighbors(flock_, d_);
  return current_boid.alignment(neighbors, a_);
};

Vector Flock::flock_cohesion(const Boid& current_boid) const {
  std::vector<Boid> neighbors = current_boid.get_neighbors(flock_, d_);
  if (neighbors.empty()) {
    return Vector(0.f, 0.f);
  }
  return current_boid.cohesion(neighbors, c_);
};

void Flock::flock_update(float delta_t) {
  for (auto& boid : flock_) {
    std::vector<Boid> neighbors = boid.get_neighbors(flock_, d_);

    Vector delta_v(0.f, 0.f);
    if (!neighbors.empty()) {
      delta_v += flock_separation(boid);
      delta_v += flock_alignment(boid);
      delta_v += flock_cohesion(boid);
    }

    boid.change_vel(delta_v);
    boid.speed_limit(vel_max, vel_min);
    boid.change_pos(boid.get_vel() * delta_t);
  }
}

FlockStats Flock::state() const {
  FlockStats stats{0.f, 0.f, 0.f, 0.f};

  const size_t N = flock_.size();
  if (N < 2) {
    return {0., 0., 0., 0.};
  }

  // statistiche sulle distanze
  float sum_dist = 0.f;
  float sum_dist2 = 0.f;
  size_t n_pairs = 0;

  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      float d = flock_[i].get_pos().distance(flock_[j].get_pos());

      sum_dist += d;
      sum_dist2 += d * d;
      ++n_pairs;
    }
  }

  stats.avg_distance = sum_dist / static_cast<float>(n_pairs);
  float var_dist =
      sum_dist2 / static_cast<float>(n_pairs) -
      stats.avg_distance * stats.avg_distance;  // calcolo la varianza
  stats.dev_distance = std::sqrt(var_dist);

  // statistiche sulle velocità
  float sum_vel = 0.f;
  float sum_vel2 = 0.f;

  for (const auto& boid : flock_) {
    float v = boid.get_vel().norm();
    sum_vel += v;
    sum_vel2 += v * v;
  }
  stats.avg_velocity = sum_vel / static_cast<float>(N);
  float var_vel = sum_vel2 / static_cast<float>(N) -
                  stats.avg_velocity * stats.avg_velocity;
  stats.dev_velocity = std::sqrt(var_vel);

  return stats;
}
}  // namespace math