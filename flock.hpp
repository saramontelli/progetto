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

 public:
  Flock(const float d, const float ds, const float s, const float a,
        const float c, const float max_speed, const float min_speed);
  void add_boids(const Boid& new_boid);

 const std::vector<Boid>& get_flock() const;

  Vector flock_separation(const Boid& current_boid) const;
  Vector flock_alignment(const Boid& current_boid) const;
  Vector flock_cohesion(const Boid& current_boid) const;

  void flock_update(float delta_t);
  FlockStats state() const;
};

}  // namespace math

#endif