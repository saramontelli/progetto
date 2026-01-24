#include <iostream>
#include <random>

#include "boid.hpp"
#include "flock.hpp"
#include "vector.hpp"

int main() {
  std::cout << "Insert the closeness parameter (values permitted are between "
               "[60,100]) : \n";
  float closeness_parameter;
  std::cin >> closeness_parameter;
  if (closeness_parameter < 60.f || closeness_parameter > 100.f) {
    std::cerr << "Error: closeness parameter out of range. \n";
    return 1;
  }
  std::cout << "Insert the distance of separation (values permitted are "
               "between [15,30]): \n";
  float distance_of_separation;
  std::cin >> distance_of_separation;
  if (distance_of_separation < 15.f || distance_of_separation > 30.f) {
    std::cerr << "Error: distance of separation out of range. \n";
    return 1;
  }
  std::cout << "Insert the separation parameter (values permitted are between "
               "[0.05,0.3]): \n";
  float separation_parameter;
  std::cin >> separation_parameter;
  if (separation_parameter < 0.05f || separation_parameter > 0.3f) {
    std::cerr << "Error: separation parameter out of range. \n";
    return 1;
  }
  std::cout << "Insert the alignment parameter (values permitted are between "
               "[0.01,0.1]): \n";
  float alignment_parameter;
  std::cin >> alignment_parameter;
  if (alignment_parameter < 0.01f || alignment_parameter > 0.1f) {
    std::cerr << "Error: alignment parameter out of range. \n";
    return 1;
  }

  std::cout << "Insert cohesion_parameter (values permitted are between "
               "[0.005, 0.05]): \n";
  float cohesion_parameter;
  std::cin >> cohesion_parameter;
  if (cohesion_parameter < 0.005f || cohesion_parameter > 0.05f) {
    std::cerr << "Error: cohesion parameter out of range. \n";
    return 1;
  }

  math::Flock flock(closeness_parameter, distance_of_separation,
                    separation_parameter, alignment_parameter,
                    cohesion_parameter, 30.0f, 5.0f);

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<float> pos_dist(0.f, 1000.0f);
  std::uniform_real_distribution<float> vel_dist(-5.0f, 5.0f);

  int N_boids;
  std::cout << "Insert the number of boids: ";
  std::cin >> N_boids;
  if (N_boids < 1) {
    std::cerr << "Error: number of boids must be >=1 \n";
    return 1;
  }

  for (int i = 0; i < N_boids; ++i) {
    math::Vector position{pos_dist(gen), pos_dist(gen)};
    math::Vector velocity{vel_dist(gen), vel_dist(gen)};
    math::Boid boid(position, velocity);
    flock.add_boids(boid);
  }

  float delta_t = 0.01f;
  int N_steps = 1000;

  for (int step = 0; step < N_steps; ++step) {
    flock.flock_update(delta_t);
    if (step % 50 == 0) {
      auto stats = flock.state();
      std::cout << "Step" << step << ": avg_distance = " << stats.avg_distance
                << "+/-" << stats.dev_distance
                << ", avg_velocity = " << stats.avg_velocity << "+/-"
                << stats.dev_velocity << "\n";
    }
  }
  std::cout << "Simulation finished. \n";
  return 0;
}