#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boid.hpp"

namespace math {
struct FlockStats {
  float avg_distance;
  float dev_distance;
  float avg_velocity;
  float dev_velocity;
};

class Flock {
 private:
  const float d_;
  const float d_s_;
  const float s_;
  const float a_;
  const float c_;
  const float vel_max;
  const float vel_min;
  std::vector<Boid> flock_;
  std::vector<Boid> predators_;

 public:
  Flock(const float d, const float ds, const float s, const float a,
        const float c, const float max_speed, const float min_speed);
  void add_boids(const Boid& new_boid);

 const std::vector<Boid>& get_flock() const;
 const std::vector<Boid>& get_predators() const;

  Vector flock_separation(const Boid& current_boid, const std::vector<Boid>& neighbors, float x_max, float y_max) const;
  Vector flock_alignment(const Boid& current_boid, const std::vector<Boid>& flock_neighbors) const;
  Vector flock_cohesion(const Boid& current_boid, const std::vector<Boid>& flock_neighbors, float x_max, float y_max) const;

  Vector avoid_predators(const Boid&, float x_max, float y_max) const;
  Vector chase_prey(const Boid&, const std::vector<Boid>& flock_neighbors, float x_max, float y_max);

  void predators_update(float delta_t, float x_max, float y_max);
  void flock_update(float delta_t, float x_max, float y_max);
  FlockStats state(float x_max, float y_max) const;
};

}  // namespace math

#endif