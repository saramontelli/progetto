#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

#include "boid.hpp"
#include "flock.hpp"
#include "vector.hpp"


float to_degrees(float radians) { return radians * 180.f / 3.14159265f; }
int main() {
  std::cout << "Boid Simulation, instructions:\n";
  std::cout << "1. Click B button to add a Boid.\n";
  std::cout << "2. Click P button to add a predator.\n";
  std::cout << "3. Finally, close the window to stop the simulation.\n";

  const float width = 1200.f;
  const float height = 800.f;

  std::cout << "Insert the following parameters: \n";
  std::cout << "Insert the closeness parameter (values permitted are between "
               "[200,300]) : \n";
  float closeness_parameter;
  std::cin >> closeness_parameter;
  if (closeness_parameter < 200.f || closeness_parameter > 300.f) {
    std::cerr << "Error: closeness parameter out of range. \n";
    return 1;
  }
  std::cout << "Insert the distance of separation (values permitted are "
               "between [40,50]): \n";
  float distance_of_separation;
  std::cin >> distance_of_separation;
  if (distance_of_separation < 40.f || distance_of_separation > 50.f) {
    std::cerr << "Error: distance of separation out of range. \n";
    return 1;
  }
  std::cout << "Insert the separation parameter (values permitted are between "
               "[1.0, 2.0]): \n";
  float separation_parameter;
  std::cin >> separation_parameter;
  if (separation_parameter < 1.f || separation_parameter > 2.f) {
    std::cerr << "Error: separation parameter out of range. \n";
    return 1;
  }
  std::cout << "Insert the alignment parameter (values permitted are between "
               "[0.3,0.7]): \n";
  float alignment_parameter;
  std::cin >> alignment_parameter;
  if (alignment_parameter < 0.3f || alignment_parameter > 0.7f) {
    std::cerr << "Error: alignment parameter out of range. \n";
    return 1;
  }

  std::cout << "Insert cohesion_parameter (values permitted are between "
               "[0.01, 0.04]): \n";
  float cohesion_parameter;
  std::cin >> cohesion_parameter;
  if (cohesion_parameter < 0.01f || cohesion_parameter > 0.04f) {
    std::cerr << "Error: cohesion parameter out of range. \n";
    return 1;
  }
  std::cout << "Insert number of predators (values permitted are between 0 and "
               "5): \n";
  int num_predators;
  std::cin >> num_predators;
  if (num_predators < 0 || num_predators > 5) {
    std::cerr << "Error: predators number out of range. \n";
    return 1;
  }

  math::Flock flock(closeness_parameter, distance_of_separation,
                    separation_parameter, alignment_parameter,
                    cohesion_parameter, 150.0f, 70.0f);

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<float> x_dist(0.f, width);
  std::uniform_real_distribution<float> y_dist(0.f, height);
  std::uniform_real_distribution<float> vel_dist(-150.0f, 150.0f);

  int N_boids;
  std::cout << "Insert the number of boids: ";
  std::cin >> N_boids;
  if (N_boids < 1) {
    std::cerr << "Error: number of boids must be >=1 \n";
    return 1;
  }

  for (int i = 0; i < N_boids; ++i) {
    math::Vector pos_b{x_dist(gen), y_dist(gen)};
    math::Vector vel_b{vel_dist(gen), vel_dist(gen)};
    math::Boid boid(pos_b, vel_b, false);
    flock.add_boids(boid);
  }

  for (int i = 0; i < num_predators; ++i) {
    math::Vector pos_p(x_dist(gen), y_dist(gen));
    math::Vector vel_p(vel_dist(gen), vel_dist(gen));
    math::Boid predator(pos_p, vel_p, true);
    flock.add_boids(predator);
  }

  sf::ContextSettings settings;
  settings.antialiasingLevel = 8;
  // 1. Inizializzazione Finestra
  sf::RenderWindow window(sf::VideoMode(1000, 800), "Boids Simulation",
                          sf::Style::Default, settings);
  window.setFramerateLimit(60);

  // 2. Definizione della forma del Boid (Triangolo)
  sf::ConvexShape boidShape;
  boidShape.setPointCount(3);
  boidShape.setPoint(0, sf::Vector2f(20.f, 0.f));     // punta
  boidShape.setPoint(1, sf::Vector2f(-10.f, 10.f));   // base sinistra
  boidShape.setPoint(2, sf::Vector2f(-10.f, -10.f));  // base destra

  boidShape.setScale(0.5f, 0.5f);
  boidShape.setFillColor(sf::Color::Cyan);

  sf::ConvexShape predatorShape = boidShape;
  predatorShape.setFillColor(sf::Color::Red);

  sf::Clock clock;
  int step = 0;
  while (window.isOpen()) {
    float delta_t = clock.restart().asSeconds();

    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
      if (event.type == sf::Event::KeyPressed) {
        math::Vector random_pos{x_dist(gen), y_dist(gen)};

        math::Vector random_vel{vel_dist(gen), vel_dist(gen)};

        if (event.key.code == sf::Keyboard::B) {
          math::Boid newBoid(random_pos, random_vel, false);
          flock.add_boids(newBoid);
          std::cout << "Added new boid" << "\n";
        }

        if (event.key.code == sf::Keyboard::P) {
          if (flock.get_predators().size() < 5) {
            flock.add_boids(math::Boid(random_pos, random_vel, true));
            std::cout << "Added new predator. Predators number: "
                      << flock.get_predators().size() << "/5" << std::endl;
          } else {
            std::cout << "Maximum number of predator reached!" << std::endl;
          }
        }
      }
    }
    flock.flock_update(delta_t, width, height);
    flock.predators_update(delta_t, width, height);

    if (step % 100 == 0) {  // Stampa ogni 100 frame per non intasare la console
      auto stats = flock.state(width, height);
      std::cout << "Step " << step << ": avg_distance = " << stats.avg_distance
                << " +/- " << stats.dev_distance
                << ", avg_velocity = " << stats.avg_velocity << " +/- "
                << stats.dev_velocity << "\n";
    }
    step++;
    window.clear(sf::Color(30, 30, 30));  // Sfondo scuro

    for (const auto& b : flock.get_flock()) {
      boidShape.setPosition(b.get_pos().get_x(), b.get_pos().get_y());

      float angle = std::atan2(b.get_vel().get_y(), b.get_vel().get_x());
      boidShape.setRotation(to_degrees(angle));

      window.draw(boidShape);
    }

    for (const auto& p : flock.get_predators()) {
      predatorShape.setPosition(p.get_pos().get_x(), p.get_pos().get_y());

      float angle = std::atan2(p.get_vel().get_y(), p.get_vel().get_x());
      predatorShape.setRotation(to_degrees(angle));

      window.draw(predatorShape);
    }

    window.display();
  }

  std::cout << "Simulation finished. \n";
  return 0;
}