#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boid.hpp"

#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid Class") {
  const math::Vector r1(0.f, 0.f);
  const math::Vector v1(5.f, 1.f);

  const math::Vector r2(2.f, 1.f);
  const math::Vector v2(-1.f, 2.5f);

  const math::Vector r3(4.f, -2.f);
  const math::Vector v3(15.f, 0.5f);

  const math::Vector r4(8.f, 9.f);
  const math::Vector v4(-3.5f, -5.f);

  math::Boid boid1(r1, v1);
  math::Boid boid2(r2, v2);
  math::Boid boid3(r3, v3);
  math::Boid boid4(r4, v4);

  std::vector<math::Boid> boids = {boid1, boid2, boid3, boid4};
  std::vector<math::Boid> near1 = boid1.get_neighbors(boids, 5.f);
  std::vector<math::Boid> near2 = boid2.get_neighbors(boids, 2.f);
  std::vector<math::Boid> near3 = boid3.get_neighbors(boids, 10.f);
  std::vector<math::Boid> near4 = boid4.get_neighbors(boids, 1.5f);

  SUBCASE("Testing Neighboring function") {
    CHECK(boid1.get_neighbors(boids, 5.f).size() == 2);
    CHECK(boid2.get_neighbors(boids, 2.f).size() == 0);
    CHECK(boid3.get_neighbors(boids, 10.f).size() == 2);
    CHECK(boid4.get_neighbors(boids, 1.5f).size() == 0);
  }

  SUBCASE("Testing separation") {
    CHECK(boid1.separation(near1, 0.5f, 5.f).get_x() ==
          doctest::Approx(-3.f).epsilon(0.1));
    CHECK(boid1.separation(near1, 0.5f, 5.f).get_y() ==
          doctest::Approx(0.5f).epsilon(0.1));
    CHECK(boid2.separation(near2, 2.f, 2.f).get_x() ==
          doctest::Approx(0.f).epsilon(0.1));
    CHECK(boid2.separation(near2, 2.f, 2.f).get_y() ==
          doctest::Approx(0.f).epsilon(0.1));
    CHECK(boid3.separation(near3, 2.f, 10.f).get_x() ==
          doctest::Approx(12.f).epsilon(0.1));
    CHECK(boid3.separation(near3, 2.f, 10.f).get_y() ==
          doctest::Approx(-10.f).epsilon(0.1));
    CHECK(boid4.separation(near4, 1.5f, 1.5f).get_x() ==
          doctest::Approx(0.f).epsilon(0.1));
    CHECK(boid4.separation(near4, 1.5f, 1.5f).get_y() ==
          doctest::Approx(0.f).epsilon(0.1));
  }

  SUBCASE("testing alignment") {
    CHECK(boid1.alignment(near1, 0.5f).get_x() ==
          doctest::Approx(1.f).epsilon(0.001));
    CHECK(boid1.alignment(near1, 0.5f).get_y() ==
          doctest::Approx(0.25f).epsilon(0.001));
    CHECK(boid2.alignment(near2, 0.5f).get_x() ==
          doctest::Approx(0.f).epsilon(0.001));
    CHECK(boid2.alignment(near2, 0.5f).get_y() ==
          doctest::Approx(0.f).epsilon(0.001));
    CHECK(boid3.alignment(near3, 0.5f).get_x() ==
          doctest::Approx(-6.5f).epsilon(0.001));
    CHECK(boid3.alignment(near3, 0.5f).get_y() ==
          doctest::Approx(0.625f).epsilon(0.001));
    CHECK(boid4.alignment(near4, 0.5f).get_x() ==
          doctest::Approx(0.f).epsilon(0.001));
    CHECK(boid4.alignment(near4, 0.5f).get_y() ==
          doctest::Approx(0.f).epsilon(0.001));
  }

  SUBCASE("Testing cohesion") {
    CHECK(boid1.cohesion(near1, 0.5f).get_x() ==
          doctest::Approx(1.5f).epsilon(0.001));
    CHECK(boid1.cohesion(near1, 0.5f).get_y() ==
          doctest::Approx(-0.25f).epsilon(0.001));
    CHECK(boid2.cohesion(near2, 0.5f).get_x() ==
          doctest::Approx(0.f).epsilon(0.001));
    CHECK(boid2.cohesion(near2, 0.5f).get_y() ==
          doctest::Approx(0.f).epsilon(0.001));
    CHECK(boid3.cohesion(near3, 0.5f).get_x() ==
          doctest::Approx(-1.5f).epsilon(0.001));
    CHECK(boid3.cohesion(near3, 0.5f).get_y() ==
          doctest::Approx(1.25f).epsilon(0.001));
    CHECK(boid4.cohesion(near4, 0.5f).get_x() ==
          doctest::Approx(0.f).epsilon(0.001));
    CHECK(boid4.cohesion(near4, 0.5f).get_y() ==
          doctest::Approx(0.f).epsilon(0.001));
  }

  SUBCASE("testing get_...()") {
    CHECK(boid1.get_pos().get_x() == doctest::Approx(0.f));
    CHECK(boid1.get_pos().get_y() == doctest::Approx(0.f));
    CHECK(boid2.get_pos().get_x() == doctest::Approx(2.f));
    CHECK(boid2.get_pos().get_y() == doctest::Approx(1.f));
    CHECK(boid3.get_pos().get_x() == doctest::Approx(4.f));
    CHECK(boid3.get_pos().get_y() == doctest::Approx(-2.f));
    CHECK(boid4.get_pos().get_x() == doctest::Approx(8.f));
    CHECK(boid4.get_pos().get_y() == doctest::Approx(9.f));
    CHECK(boid1.get_vel().get_x() == doctest::Approx(5.f));
    CHECK(boid1.get_vel().get_y() == doctest::Approx(1.f));
    CHECK(boid2.get_vel().get_x() == doctest::Approx(-1.f));
    CHECK(boid2.get_vel().get_y() == doctest::Approx(2.5f));
    CHECK(boid3.get_vel().get_x() == doctest::Approx(15.f));
    CHECK(boid3.get_vel().get_y() == doctest::Approx(0.5f));
    CHECK(boid4.get_vel().get_x() == doctest::Approx(-3.5f));
    CHECK(boid4.get_vel().get_y() == doctest::Approx(-5.f));
  }
}

TEST_CASE("Testing == operator") {
  SUBCASE("equal component") {
    const math::Vector r(3.f, 4.f);
    const math::Vector v(1.f, 2.f);
    math::Boid boid1(r, v);
    math::Boid boid2(r, v);
    CHECK(boid1.operator==(boid2) == true);
  }

  SUBCASE("Non-equal component") {
    const math::Vector r1(0.f, 0.f);
    const math::Vector v1(5.f, 1.f);
    const math::Vector r2(2.f, 3.f);
    const math::Vector v2(5.f, 1.f);
    math::Boid boid1(r1, v1);
    math::Boid boid2(r2, v2);
    CHECK(boid1.operator==(boid2) == false);
  }
}