#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("testing flock class") {
  const float x_max = 50.f;
  const float y_max = 50.f;

  const math::Vector r1(0.f, 0.f);
  const math::Vector v1(1.f, 0.f);

  const math::Vector r2(10.f, 5.f);
  const math::Vector v2(0.f, 1.f);

  const math::Vector r3(20.f, -5.f);
  const math::Vector v3(-1.f, 0.f);

  const math::Vector r4(5.f, -5.f);
  const math::Vector v4(0.f, -1.f);

  const math::Boid boid1(r1, v1, 0);
  const math::Boid boid2(r2, v2, 0);
  const math::Boid boid3(r3, v3, 0);
  const math::Boid boid4(r4, v4, 1);

  const float d = 50.f;
  const float ds = 15.f;
  const float s = 0.5f;
  const float a = 0.3f;
  const float c = 0.1f;
  const float max_speed = 10.f;
  const float min_speed = 0.1f;

  SUBCASE("Testing add_boids and getters") {
    math::Flock flock(d, ds, s, a, c, max_speed, min_speed);
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    CHECK(flock.get_flock().size() == 3);
    CHECK(flock.get_predators().size() == 1);

    CHECK(flock.get_flock()[0] == boid1);
    CHECK(flock.get_flock()[1] == boid2);
    CHECK(flock.get_flock()[2] == boid3);

    CHECK(flock.get_predators()[0] == boid4);
  }

  SUBCASE("Testing flock_separation") {
    math::Flock flock(d, ds, s, a, c, max_speed, min_speed);
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    auto neighbors_b1 = boid1.get_neighbors(flock.get_flock(), d, x_max, y_max);

    auto neighbors_b2 = boid2.get_neighbors(flock.get_flock(), d, x_max, y_max);

    auto neighbors_b3 = boid3.get_neighbors(flock.get_flock(), d, x_max, y_max);

    math::Vector sep1 =
        flock.flock_separation(boid1, neighbors_b1, x_max, y_max);
    math::Vector sep2 =
        flock.flock_separation(boid2, neighbors_b2, x_max, y_max);
    math::Vector sep3 =
        flock.flock_separation(boid3, neighbors_b3, x_max, y_max);

    CHECK(sep1.get_x() == doctest::Approx(-5.0).epsilon(0.1));
    CHECK(sep1.get_y() == doctest::Approx(-2.5).epsilon(0.1));

    CHECK(sep2.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(sep2.get_y() == doctest::Approx(7.5).epsilon(0.1));

    CHECK(sep3.get_x() == doctest::Approx(5.0).epsilon(0.1));
    CHECK(sep3.get_y() == doctest::Approx(-5.0).epsilon(0.1));
  }

  SUBCASE("Testing flock_alignment method") {
    math::Flock flock(d, ds, s, a, c, max_speed, min_speed);
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    auto neighbors_b1 = boid1.get_neighbors(flock.get_flock(), d, x_max, y_max);
    auto neighbors_b2 = boid2.get_neighbors(flock.get_flock(), d, x_max, y_max);
    auto neighbors_b3 = boid3.get_neighbors(flock.get_flock(), d, x_max, y_max);

    math::Vector alig1 = flock.flock_alignment(boid1, neighbors_b1);
    CHECK(alig1.get_x() == doctest::Approx(-0.45).epsilon(0.01));
    CHECK(alig1.get_y() == doctest::Approx(0.15).epsilon(0.01));

    math::Vector alig2 = flock.flock_alignment(boid2, neighbors_b2);
    CHECK(alig2.get_x() == doctest::Approx(0.0).epsilon(0.01));
    CHECK(alig2.get_y() == doctest::Approx(-0.3).epsilon(0.01));

    math::Vector alig3 = flock.flock_alignment(boid3, neighbors_b3);
    CHECK(alig3.get_x() == doctest::Approx(0.45).epsilon(0.01));
    CHECK(alig3.get_y() == doctest::Approx(0.15).epsilon(0.01));
  }

  SUBCASE("Testing flock_cohesion") {
    math::Flock flock(d, ds, s, a, c, max_speed, min_speed);
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    auto neighbors_b1 = boid1.get_neighbors(flock.get_flock(), d, x_max, y_max);
    auto neighbors_b2 = boid2.get_neighbors(flock.get_flock(), d, x_max, y_max);
    auto neighbors_b3 = boid3.get_neighbors(flock.get_flock(), d, x_max, y_max);

    math::Vector coh1 = flock.flock_cohesion(boid1, neighbors_b1, x_max, y_max);
    CHECK(coh1.get_x() == doctest::Approx(1.5).epsilon(0.1));
    CHECK(coh1.get_y() == doctest::Approx(0.0).epsilon(0.1));

    math::Vector coh2 = flock.flock_cohesion(boid2, neighbors_b2, x_max, y_max);
    CHECK(coh2.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(coh2.get_y() == doctest::Approx(-0.75).epsilon(0.1));

    math::Vector coh3 = flock.flock_cohesion(boid3, neighbors_b3, x_max, y_max);
    CHECK(coh3.get_x() == doctest::Approx(-1.5).epsilon(0.1));
    CHECK(coh3.get_y() == doctest::Approx(0.75).epsilon(0.1));
  }

  SUBCASE("Testing avoid_predators method") {
    math::Flock flock(d, ds, s, a, c, max_speed, min_speed);

    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    math::Vector avoid1 = flock.avoid_predators(boid1, x_max, y_max);
    math::Vector avoid2 = flock.avoid_predators(boid2, x_max, y_max);
    math::Vector avoid3 = flock.avoid_predators(boid3, x_max, y_max);

    CHECK(avoid1.get_x() == doctest::Approx(-25.f).epsilon(0.1));
    CHECK(avoid1.get_y() == doctest::Approx(25.f).epsilon(0.1));

    CHECK(avoid2.get_x() == doctest::Approx(25.f).epsilon(0.1));
    CHECK(avoid2.get_y() == doctest::Approx(50.f).epsilon(0.1));

    CHECK(avoid3.get_x() == doctest::Approx(0.0f).epsilon(0.1));
    CHECK(avoid3.get_y() == doctest::Approx(0.0f).epsilon(0.1));
  }

  SUBCASE("Testing chase_prey method") {
    math::Flock flock(d, ds, s, a, c, max_speed, min_speed);
    flock.add_boids(boid1);
    flock.add_boids(boid2);
    flock.add_boids(boid3);
    flock.add_boids(boid4);

    auto neighbors_pred =
        boid4.get_neighbors(flock.get_flock(), d, x_max, y_max);
    math::Vector chase_vec =
        flock.chase_prey(boid4, neighbors_pred, x_max, y_max);

    CHECK(chase_vec.get_x() == doctest::Approx(-0.5).epsilon(0.1));
    CHECK(chase_vec.get_y() == doctest::Approx(0.5).epsilon(0.1));
  }

  SUBCASE("Testing flock state") {
    math::Flock flock_stat(d, ds, s, a, c, max_speed, min_speed);

    flock_stat.add_boids(boid1);
    auto status1 = flock_stat.state(x_max, y_max);
    CHECK(status1.avg_distance == doctest::Approx(0.0f).epsilon(0.1));
    CHECK(status1.dev_distance == doctest::Approx(0.0f).epsilon(0.1));
    CHECK(status1.avg_velocity == doctest::Approx(0.0f).epsilon(0.1));
    CHECK(status1.dev_velocity == doctest::Approx(0.0f).epsilon(0.1));

    flock_stat.add_boids(boid2);
    flock_stat.add_boids(boid3);
    flock_stat.add_boids(boid4);

    auto status4 = flock_stat.state(x_max, y_max);
    CHECK(status4.avg_distance == doctest::Approx(15.31f).epsilon(0.1));
    CHECK(status4.dev_distance == doctest::Approx(3.95f).epsilon(0.1));
    CHECK(status4.avg_velocity == doctest::Approx(1.0f).epsilon(0.1));
    CHECK(status4.dev_velocity == doctest::Approx(0.0f).epsilon(0.1));
  }
}

TEST_CASE("testing flock_update") {
  const float x_max = 50.f;
  const float y_max = 50.f;
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

  math::Boid boid1(r1, v1, 0);
  math::Boid boid2(r2, v2, 0);
  math::Boid boid3(r3, v3, 0);
  math::Boid boid4(r4, v4, 0);
  math::Boid boid5(r5, v5, 0);

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
  flock.flock_update(delta_t, x_max, y_max);

  auto boids_result = flock.get_flock();

  CHECK(boids_result[0].get_vel().get_x() ==
        doctest::Approx(-6.46f).epsilon(0.01));
  CHECK(boids_result[0].get_vel().get_y() ==
        doctest::Approx(-3.16f).epsilon(0.01));
  CHECK(boids_result[0].get_pos().get_x() ==
        doctest::Approx(43.54f).epsilon(0.01));
  CHECK(boids_result[0].get_pos().get_y() ==
        doctest::Approx(46.84f).epsilon(0.01));

  CHECK(boids_result[1].get_vel().get_x() ==
        doctest::Approx(2.91f).epsilon(0.01));
  CHECK(boids_result[1].get_vel().get_y() ==
        doctest::Approx(9.33f).epsilon(0.01));
  CHECK(boids_result[1].get_pos().get_x() ==
        doctest::Approx(12.91f).epsilon(0.01));
  CHECK(boids_result[1].get_pos().get_y() ==
        doctest::Approx(14.33f).epsilon(0.01));

  CHECK(boids_result[2].get_vel().get_x() ==
        doctest::Approx(3.54f).epsilon(0.01));
  CHECK(boids_result[2].get_vel().get_y() ==
        doctest::Approx(-4.04f).epsilon(0.01));
  CHECK(boids_result[2].get_pos().get_x() ==
        doctest::Approx(23.54f).epsilon(0.01));
  CHECK(boids_result[2].get_pos().get_y() ==
        doctest::Approx(40.96f).epsilon(0.01));

  CHECK(boids_result[3].get_vel().get_x() ==
        doctest::Approx(-0.84f).epsilon(0.01));
  CHECK(boids_result[3].get_vel().get_y() ==
        doctest::Approx(-1.54f).epsilon(0.01));
  CHECK(boids_result[3].get_pos().get_x() ==
        doctest::Approx(29.16f).epsilon(0.01));
  CHECK(boids_result[3].get_pos().get_y() ==
        doctest::Approx(8.46f).epsilon(0.01));

  CHECK(boids_result[4].get_vel().get_x() ==
        doctest::Approx(1.35f).epsilon(0.01));
  CHECK(boids_result[4].get_vel().get_y() ==
        doctest::Approx(-0.1f).epsilon(0.01));
  CHECK(boids_result[4].get_pos().get_x() ==
        doctest::Approx(6.35f).epsilon(0.01));
  CHECK(boids_result[4].get_pos().get_y() ==
        doctest::Approx(1.9f).epsilon(0.01));
}

TEST_CASE("testing predators_update") {
  const float x_max = 50.f;
  const float y_max = 50.f;
  const float delta_t = 1.f;

  const math::Vector r1(0.f, 0.f);
  const math::Vector r2(10.f, 0.f);
  const math::Vector r3(20.f, 0.f);
  const math::Vector rp1(5.f, 5.f);

  const math::Vector v1(1.f, 0.f);
  const math::Vector v2(1.f, 0.f);
  const math::Vector v3(1.f, 0.f);
  const math::Vector vp1(0.f, 0.f);

  math::Boid boid1(r1, v1, 0);
  math::Boid boid2(r2, v2, 0);
  math::Boid boid3(r3, v3, 0);
  math::Boid predator1(rp1, vp1, 1);

  const float d = 50.f;
  const float ds = 15.f;
  const float s = 0.5f;
  const float a = 0.3f;
  const float c = 0.1f;
  const float max_speed = 10.f;
  const float min_speed = 0.1f;

  math::Flock flock(d, ds, s, a, c, max_speed, min_speed);

  flock.add_boids(boid1);
  flock.add_boids(boid2);
  flock.add_boids(boid3);
  flock.add_boids(predator1);

  flock.predators_update(delta_t, x_max, y_max);

  auto pred_updated = flock.get_predators()[0];

  CHECK(pred_updated.get_vel().get_x() == doctest::Approx(-0.5).epsilon(0.01));
  CHECK(pred_updated.get_vel().get_y() == doctest::Approx(-0.5).epsilon(0.01));
  CHECK(pred_updated.get_pos().get_x() == doctest::Approx(4.5).epsilon(0.01));
  CHECK(pred_updated.get_pos().get_y() == doctest::Approx(4.5).epsilon(0.01));
}
