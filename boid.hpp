#ifndef BOID_HPP
#define BOID_HPP
#include <vector>

#include "vector.hpp"

namespace math {

class Boid {
 private:
  Vector pos_;
  Vector vel_;

 public:
  Boid();
  Boid(Vector position, Vector velocity);

  Vector get_pos() const;
  Vector get_vel() const;
  void set_vel(const Vector& v);

  bool operator==(const Boid&) const;


  // forza che allontana dai boids vicini
  Vector separation(const std::vector<Boid>& boids, float s, float d_s) const;
  // il boids si deve allineare con quelli vicini
  Vector alignment(const std::vector<Boid>& boids, float a) const;
  // boid si muove verso il baricentro dei boids vicini
  Vector cohesion(const std::vector<Boid>& boids, float c) const;
  // boids nelle vicinanze
  std::vector<Boid> get_neighbors(const std::vector<Boid>& all_boids,
                                  float d) const;

  void change_vel(const Vector&);
  void change_pos(const Vector&);
};
}  // namespace math

#endif