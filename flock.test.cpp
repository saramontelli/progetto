#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("testing flock class") {
  const math::Vector r1(0.f, 0.f);
  const math::Vector v1(1.f, 0.f);

  const math::Vector r2(10.f, 5.f);
  const math::Vector v2(0.f, 1.f);

  const math::Vector r3(20.f, -5.f);
  const math::Vector v3(-1.f, 0.f);

  const math::Vector r4(30.f, 10.f);
  const math::Vector v4(0.f, -1.f);

  math::Boid boid1(r1, v1);
  math::Boid boid2(r2, v2);
  math::Boid boid3(r3, v3);
  math::Boid boid4(r4, v4);

  const float d = 50.f;   // raggio vicinato
  const float ds = 15.f;  // distanza di separazione
  const float s = 0.5f;   // separazione
  const float a = 0.3f;   // allineamento
  const float c = 0.1f;   // coesione
  const float max_speed = 10.f;
  const float min_speed = 0.1f;

  math::Flock flock(d, ds, s, a, c, max_speed, min_speed);

  SUBCASE("Testing add_boids") {
    flock.add_boids(boid1);
    CHECK(flock.get_flock().size() == 1);
    flock.add_boids(boid2);
    CHECK(flock.get_flock().size() == 2);
    flock.add_boids(boid3);
    CHECK(flock.get_flock().size() == 3);
    flock.add_boids(boid4);
    CHECK(flock.get_flock().size() == 4);
  }

  SUBCASE("Testing get_flock") {
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);
    auto boids = flock.get_flock();
    CHECK(boids.size() == 4);
    CHECK(boids[0].get_pos() == r1);
    CHECK(boids[1].get_pos() == r2);
    CHECK(boids[2].get_pos() == r3);
    CHECK(boids[3].get_pos() == r4);
  }

  SUBCASE("Testing flock_separation") {
    const math::Vector rb1(0.f, 0.f);
    const math::Vector vb1(1.f, 0.f);

    const math::Vector rb2(10.f, 5.f);
    const math::Vector vb2(0.f, 1.f);

    const math::Vector rb3(20.f, -5.f);
    const math::Vector vb3(-1.f, 0.f);

    const math::Vector rb4(30.f, 10.f);
    const math::Vector vb4(0.f, -1.f);

    math::Boid b1(rb1, vb1);
    math::Boid b2(rb2, vb2);
    math::Boid b3(rb3, vb3);
    math::Boid b4(rb4, vb4);

    flock.add_boids(b1);
    flock.add_boids(b2);
    flock.add_boids(b3);
    flock.add_boids(b4);

    math::Vector sep1 = flock.flock_separation(b1);
    CHECK(sep1.get_x() == doctest::Approx(-5.0).epsilon(0.1));
    CHECK(sep1.get_y() == doctest::Approx(-2.5).epsilon(0.1));

    math::Vector sep2 = flock.flock_separation(b2);
    CHECK(sep2.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(sep2.get_y() == doctest::Approx(7.5).epsilon(0.1));

    math::Vector sep3 = flock.flock_separation(b3);
    CHECK(sep3.get_x() == doctest::Approx(5.0).epsilon(0.1));
    CHECK(sep3.get_y() == doctest::Approx(-5.0).epsilon(0.1));

    math::Vector sep4 = flock.flock_separation(b4);
    CHECK(sep4.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(sep4.get_y() == doctest::Approx(0.0).epsilon(0.1));
  }

  SUBCASE("Testing flock_alignment") {
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    math::Vector alig1 = flock.flock_alignment(boid1);
    CHECK(alig1.get_x() == doctest::Approx(-0.4).epsilon(0.1));
    CHECK(alig1.get_y() == doctest::Approx(0.0).epsilon(0.1));
    math::Vector alig2 = flock.flock_alignment(boid2);
    CHECK(alig2.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(alig2.get_y() == doctest::Approx(-0.4).epsilon(0.1));
    math::Vector alig3 = flock.flock_alignment(boid3);
    CHECK(alig3.get_x() == doctest::Approx(0.4).epsilon(0.1));
    CHECK(alig3.get_y() == doctest::Approx(0.0).epsilon(0.1));
    math::Vector alig4 = flock.flock_alignment(boid4);
    CHECK(alig4.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(alig4.get_y() == doctest::Approx(0.4).epsilon(0.1));
  }

  SUBCASE("Testing flock_cohesion") {
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    math::Vector coh1 = flock.flock_cohesion(boid1);
    CHECK(coh1.get_x() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(coh1.get_y() == doctest::Approx(0.3).epsilon(0.1));
    math::Vector coh2 = flock.flock_cohesion(boid2);
    CHECK(coh2.get_x() == doctest::Approx(0.7).epsilon(0.1));
    CHECK(coh2.get_y() == doctest::Approx(-0.3).epsilon(0.1));
    math::Vector coh3 = flock.flock_cohesion(boid3);
    CHECK(coh3.get_x() == doctest::Approx(-0.7).epsilon(0.1));
    CHECK(coh3.get_y() == doctest::Approx(1.0).epsilon(0.1));
    math::Vector coh4 = flock.flock_cohesion(boid4);
    CHECK(coh4.get_x() == doctest::Approx(-2.0).epsilon(0.1));
    CHECK(coh4.get_y() == doctest::Approx(-1.0).epsilon(0.1));
  }

  SUBCASE("Testing state") {
    math::Flock flock_stat(d, ds, s, a, c, max_speed, min_speed);

    flock_stat.add_boids(boid1);
    auto status1 = flock_stat.state();
    CHECK(status1.avg_distance == doctest::Approx(0.0).epsilon(0.1));
    CHECK(status1.dev_distance == doctest::Approx(0.0).epsilon(0.1));
    CHECK(status1.avg_velocity == doctest::Approx(0.0).epsilon(0.1));
    CHECK(status1.dev_velocity == doctest::Approx(0.0).epsilon(0.1));

    flock_stat.add_boids(boid2);
    flock_stat.add_boids(boid3);
    flock_stat.add_boids(boid4);

    auto status4 = flock_stat.state();
    CHECK(status4.avg_distance == doctest::Approx(19.4).epsilon(0.1));
    CHECK(status4.dev_distance == doctest::Approx(6.4).epsilon(0.1));
    CHECK(status4.avg_velocity == doctest::Approx(1.0).epsilon(0.1));
    CHECK(status4.dev_velocity == doctest::Approx(0.0).epsilon(0.1));
  }
}

TEST_CASE("testing flock_update") {
  const math::Vector r1(0.f, 0.f);
  const math::Vector v1(1.f, 0.f);

  const math::Vector r2(10.f, 5.f);
  const math::Vector v2(0.f, 1.f);

  const math::Vector r3(20.f, -5.f);
  const math::Vector v3(-1.f, 0.f);

  const math::Vector r4(30.f, 10.f);
  const math::Vector v4(0.f, -1.f);

  const math::Vector r5(5.f, 2.f);
  const math::Vector v5(0.5f, 0.5f);

  math::Boid boid1(r1, v1);
  math::Boid boid2(r2, v2);
  math::Boid boid3(r3, v3);
  math::Boid boid4(r4, v4);
  math::Boid boid5(r5, v5);

  const float d = 50.f;
  const float ds = 15.f;
  const float s = 0.5f;
  const float a = 0.3f;
  const float c = 0.1f;
  const float max_speed = 10.f;
  const float min_speed = 0.1f;
  const float delta_t = 1.f;

  math::Flock flock(d, ds, s, a, c, max_speed, min_speed);

  flock.add_boids(boid1);
  flock.add_boids(boid2);
  flock.add_boids(boid3);
  flock.add_boids(boid4);
  flock.add_boids(boid5);
  flock.flock_update(delta_t);

  auto boids_result = flock.get_flock();
  CHECK(boids_result[0].get_vel().get_x() ==
        doctest::Approx(-5.2).epsilon(0.01));
  CHECK(boids_result[0].get_vel().get_y() ==
        doctest::Approx(-3.2).epsilon(0.01));
  CHECK(boids_result[0].get_pos().get_x() ==
        doctest::Approx(-5.2).epsilon(0.01));
  CHECK(boids_result[0].get_pos().get_y() ==
        doctest::Approx(-3.2).epsilon(0.01));
}
