#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"

#include "doctest.h"

TEST_CASE("Testing Get_...()") {
  math::Vector w{3.14f, 5.58f};

  CHECK(w.get_x() == doctest::Approx(3.14).epsilon(0.1));
  CHECK(w.get_y() == doctest::Approx(5.58).epsilon(0.1));
}

TEST_CASE("Testing Set_...()") {
  math::Vector w{7.31f, 6.98f};
  w.set_x(4.53);
  w.set_y(6.72);

  CHECK(w.get_x() == doctest::Approx(4.53).epsilon(0.1));
  CHECK(w.get_y() == doctest::Approx(6.72).epsilon(0.1));
}

TEST_CASE("Testing operator+") {
  SUBCASE("Positive components") {
    const math::Vector w{3.4f, 0.9f};
    const math::Vector t{2.1f, 1.4f};
    const math::Vector sum{w + t};

    CHECK(sum.get_x() == doctest::Approx(5.5).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(2.3).epsilon(0.1));
  }

  SUBCASE("Negative components") {
    const math::Vector w{-1.5f, -0.7f};
    const math::Vector t{-1.4f, -0.3f};
    const math::Vector sum{w + t};

    CHECK(sum.get_x() == doctest::Approx(-2.9).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(-1.0).epsilon(0.1));
  }

  SUBCASE("Null components") {
    const math::Vector w{9.0f, 0.0f};
    const math::Vector t{0.0f, 3.2f};
    const math::Vector sum{w + t};

    CHECK(sum.get_x() == doctest::Approx(9.0).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(3.2).epsilon(0.1));
  }
}

TEST_CASE("Operator-") {
  SUBCASE("Positive Components") {
    const math::Vector w{1.3f, 3.0f};
    const math::Vector t{4.0f, 7.8f};
    const math::Vector difference{w - t};

    CHECK(difference.get_x() == doctest::Approx(-2.7).epsilon(0.1));
    CHECK(difference.get_y() == doctest::Approx(-4.8).epsilon(0.1));
  }
  SUBCASE("Negative components") {
    const math::Vector w{-5.5f, -3.3f};
    const math::Vector t{-1.0f, -1.4f};
    const math::Vector difference{w - t};

    CHECK(difference.get_x() == doctest::Approx(-4.5).epsilon(0.1));
    CHECK(difference.get_y() == doctest::Approx(-1.9).epsilon(0.1));
  }
  SUBCASE("Null components") {
    const math::Vector w{0.0f, 6.4f};
    const math::Vector t{1.3f, 0.0f};
    const math::Vector difference{w - t};

    CHECK(difference.get_x() == doctest::Approx(-1.3).epsilon(0.1));
    CHECK(difference.get_y() == doctest::Approx(6.4).epsilon(0.1));
  }
}

TEST_CASE("Operator*") {
  SUBCASE("Positive scalar") {
    const math::Vector w{2.0f, 7.5f};
    const float s = 3.0;
    const math::Vector product{w * s};

    CHECK(product.get_x() == doctest::Approx(6.0).epsilon(0.1));
    CHECK(product.get_y() == doctest::Approx(22.5).epsilon(0.1));
  }
  SUBCASE("Negative scalar") {
    const math::Vector w{0.0f, 2.0f};
    const float s = -3.0;
    const math::Vector product{w * s};

    CHECK(product.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(product.get_y() == doctest::Approx(-6.0).epsilon(0.1));
  }
  SUBCASE("Null scalar") {
    const math::Vector w{5.5f, 7.9f};
    const float s = 0.0;
    const math::Vector product{w * s};

    CHECK(product.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(product.get_y() == doctest::Approx(0.0).epsilon(0.1));
  }
}

TEST_CASE("Operator+=") {
  SUBCASE("Positive components") {
    math::Vector w{3.6f, 4.5f};
    const math::Vector t{2.2f, 6.8f};
    w += t;
    CHECK(w.get_x() == doctest::Approx(5.8).epsilon(0.1));
    CHECK(w.get_y() == doctest::Approx(11.3).epsilon(0.1));
  }
  SUBCASE("Negative components") {
    math::Vector w{-4.3f, -1.0f};
    const math::Vector t{-6.9f, -2.2f};
    w += t;

    CHECK(w.get_x() == doctest::Approx(-11.2).epsilon(0.1));
    CHECK(w.get_y() == doctest::Approx(-3.2).epsilon(0.1));
  }
  SUBCASE("Null components") {
    math::Vector w{0.0f, 9.0f};
    const math::Vector t{-3.0f, 0.0f};
    w += t;

    CHECK(w.get_x() == doctest::Approx(-3.0).epsilon(0.1));
    CHECK(w.get_y() == doctest::Approx(9.0).epsilon(0.1));
  }
}

TEST_CASE("testing operator==") {
  const math::Vector w{0.0f, 4.5f};
  const math::Vector t{0.0f, 4.5f};

  CHECK(w == t);
}

TEST_CASE("Testing dot") {
  SUBCASE("Positive components") {
    const math::Vector w{1.5f, 5.6f};
    const math::Vector t{4.2f, 1.7f};
    CHECK(w.dot(t) == doctest::Approx(15.8).epsilon(0.1));
  }
  SUBCASE("Negative components") {
    const math::Vector w{-1.4f, -3.9f};
    const math::Vector t{-4.1f, -1.2f};
    CHECK(w.dot(t) == doctest::Approx(10.4).epsilon(0.1));
  }
  SUBCASE("Null components") {
    const math::Vector w{0.0f, 3.5f};
    const math::Vector t{4.9f, 0.0f};
    CHECK(w.dot(t) == doctest::Approx(0.0).epsilon(0.1));
  }
}
TEST_CASE("Testing norm") {
  SUBCASE("Positive components") {
    const math::Vector w{3.4f, 1.2f};
    CHECK(w.norm() == doctest::Approx(3.6).epsilon(0.1));
  }
  SUBCASE("Negative components") {
    const math::Vector w{-1.4f, -5.00f};
    CHECK(w.norm() == doctest::Approx(5.2).epsilon(0.1));
  }
  SUBCASE("Null components") {
    const math::Vector w{0.0f, 0.0f};
    CHECK(w.norm() == doctest::Approx(0.0).epsilon(0.1));
  }
}

TEST_CASE("testing distance") {
  SUBCASE("Positive components") {
    const math::Vector w{1.1f, 1.4f};
    const math::Vector t{3.9f, 0.4f};

    CHECK(w.distance(t) == doctest::Approx(3.0).epsilon(0.1));
  }
  SUBCASE("Negative components") {
    const math::Vector w{-1.8f, -0.6f};
    const math::Vector t{-5.7f, -0.4f};

    CHECK(w.distance(t) == doctest::Approx(3.9).epsilon(0.1));
  }
  SUBCASE("Null components") {
    const math::Vector w{0.0f, 1.1f};
    const math::Vector t{3.2f, 0.0f};

    CHECK(w.distance(t) == doctest::Approx(3.4).epsilon(0.1));
  }
}