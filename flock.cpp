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

void Flock::add_boids(const Boid& new_boid) {
  if (new_boid.get_predator()) {
    predators_.push_back(new_boid);
  } else {
    flock_.push_back(new_boid);
  }
}

const std::vector<Boid>& Flock::get_flock() const { return flock_; };

const std::vector<Boid>& Flock::get_predators() const { return predators_; }

Vector Flock::flock_separation(const Boid& current_boid,
                               const std::vector<Boid>& flock_neighbors,
                               float x_max, float y_max) const {
  return current_boid.separation(flock_neighbors, s_, d_s_, x_max, y_max);
};

Vector Flock::flock_alignment(const Boid& current_boid,
                              const std::vector<Boid>& flock_neighbors) const {
  return current_boid.alignment(flock_neighbors, a_);
};

Vector Flock::flock_cohesion(const Boid& current_boid,
                             const std::vector<Boid>& flock_neighbors,
                             float x_max, float y_max) const {
  if (flock_neighbors.empty()) {
    return Vector(0.f, 0.f);
  }
  return current_boid.cohesion(flock_neighbors, c_, x_max, y_max);
};

Vector Flock::avoid_predators(const Boid& current_boid, float x_max,
                              float y_max) const {
  std::vector<Boid> nearby_predators;
  for (const auto& p:predators_) {
    if (current_boid.get_pos().distance(p.get_pos(), x_max, y_max) < d_ ) {
      nearby_predators.push_back(p);
    }
  }
  if (nearby_predators.empty()) return Vector(0.f, 0.f);
  float predator_s = s_ *5.0f;
  return current_boid.separation(nearby_predators, predator_s, d_s_, x_max, y_max);
}

Vector Flock::chase_prey(const Boid& predator,
                         const std::vector<Boid>& neighbors, float x_max,
                         float y_max) {
  if (neighbors.empty()) return Vector(0.f, 0.f);
  Vector predator_pos = predator.get_pos();
  size_t closest_idx = 0;
  float min_dist = neighbors[0].get_pos().distance(predator_pos, x_max, y_max);

  for (size_t i = 1; i < neighbors.size(); ++i) {
    float dist = neighbors[i].get_pos().distance(predator_pos, x_max, y_max);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  float dx = neighbors[closest_idx].get_pos().get_x() - predator_pos.get_x();
  float dy = neighbors[closest_idx].get_pos().get_y() - predator_pos.get_y();
  if (dx > x_max / 2)
    dx -= x_max;
  else if (dx < -x_max / 2)
    dx += x_max;
  if (dy > y_max / 2)
    dy -= y_max;
  else if (dy < -y_max / 2)
    dy += y_max;

  Vector direction(dx, dy);
  return direction * c_;
}

void Flock::predators_update(float delta_t, float x_max, float y_max) {
  std::vector<Boid> next_predators = predators_;
  
  for (size_t i = 0; i < predators_.size(); ++i) {
    const auto& current_p = predators_[i];

    std::vector<Boid> flock_neighbors = current_p.get_neighbors(flock_, d_ * 1.5f, x_max, y_max);
    std::vector<Boid> predators_neighbors = current_p.get_neighbors(predators_, d_, x_max, y_max);

    Vector v_chase = chase_prey(current_p, flock_neighbors, x_max, y_max);
    Vector v_sep = current_p.separation(predators_neighbors, s_, d_s_, x_max, y_max);
    Vector delta_v = v_chase + v_sep;

    next_predators[i].change_vel(delta_v); 
    next_predators[i].speed_limit(vel_max, vel_min);

    next_predators[i].change_pos(next_predators[i].get_vel() * delta_t);
    next_predators[i].wrap_position(x_max, y_max);
  }
    predators_ = next_predators;
}
void Flock::flock_update(float delta_t, float x_max, float y_max) {
  std::vector<Boid> next_flock = flock_;

  for (size_t i = 0; i < flock_.size(); ++i) {
    std::vector<Boid> neighbors =
        flock_[i].get_neighbors(flock_, d_, x_max, y_max);

    Vector delta_v(0.f, 0.f);
    if (!neighbors.empty()) {
      delta_v += flock_[i].separation(neighbors, s_, d_s_, x_max, y_max);
      delta_v += flock_[i].alignment(neighbors, a_);
      delta_v += flock_[i].cohesion(neighbors, c_, x_max, y_max);
    }
std::vector<Boid> nearby_predators;
for (const auto& p:predators_) {
  if (flock_[i].get_pos().distance(p.get_pos(), x_max, y_max)<d_) {
nearby_predators.push_back(p);
  }
}
if (!nearby_predators.empty()) {
  delta_v += flock_[i].separation(nearby_predators, s_*10.f, d_s_, x_max, y_max);
}

    next_flock[i].change_vel(delta_v);
    next_flock[i].speed_limit(vel_max, vel_min);
    next_flock[i].change_pos(next_flock[i].get_vel() * delta_t);
    next_flock[i].wrap_position(x_max, y_max);
  }
  flock_ = next_flock;
}

FlockStats Flock::state(float x_max, float y_max) const {
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
      float d = flock_[i].get_pos().distance(flock_[j].get_pos(), x_max, y_max);

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