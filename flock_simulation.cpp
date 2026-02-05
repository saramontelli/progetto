#include <fstream>
#include <iostream>

#include "flock.hpp"
int main() {
  const float world_X = 800.0f;
  const float world_Y = 600.0f;
  std::cout << "Boid Simulation: 5 Boids starting at the edges, moving toward "
               "the center.\n";
  math::Flock flock(500.0f, 50.0f, 0.05f, 0.5f, 0.05f, 25.0f, 10.0f);
  math::Vector pos1(300.0f, 200.0f);
  math::Vector pos2(500.0f, 200.0f);
  math::Vector pos3(300.0f, 400.0f);
  math::Vector pos4(500.0f, 400.0f);
  math::Vector pos5(400.0f, 300.0f);
  math::Vector vel1(14.4f, 10.8f);
  math::Vector vel2(-17.6f, 13.2f);
  math::Vector vel3(16.0f, -12.0f);
  math::Vector vel4(-19.2f, -14.4f);
  math::Vector vel5(16.0f, 0.0f);
  math::Boid boid_1(pos1, vel1, 0);
  math::Boid boid_2(pos2, vel2, 0);
  math::Boid boid_3(pos3, vel3, 0);
  math::Boid boid_4(pos4, vel4, 0);
  math::Boid boid_5(pos5, vel5, 0);
  flock.add_boids(boid_1);
  flock.add_boids(boid_2);
  flock.add_boids(boid_3);
  flock.add_boids(boid_4);
  flock.add_boids(boid_5);
  std::ofstream output_file("flock_simulation.txt");
  output_file << "Time Mean Distance Standard Deviation of Distance Mean "
                 "Speed Standard Deviation of Speed\n";
  float time_passed = 0.0f;
  float simulation_time = 30.0f;
  float delta_t = 0.01f;
  float last_save = -0.1f;
  while (time_passed < simulation_time) {
    time_passed += delta_t;
    flock.flock_update(delta_t, world_X, world_Y);
    
    if (time_passed - last_save >= 0.5f) {
      const math::FlockStats flock_state = flock.state(world_X, world_Y);
      output_file << time_passed << " " << flock_state.avg_distance << " "
                  << flock_state.dev_distance << " " << flock_state.avg_velocity
                  << " " << flock_state.dev_velocity << "\n";
      last_save = time_passed;
    }
  }
  output_file.close();
  std::cout << "Data saved in simulation_data.txt.\n";
  return 0;
}