#ifndef VECTOR_HPP
#define VECTOR_HPP

namespace math {
class Vector {
 private:
  float x_;
  float y_;

 public:
  Vector();
  Vector(float x, float y);

  float get_x() const;
  float get_y() const;

  void set_x(float newX);
  void set_y(float newY);

  Vector operator+(const Vector& v) const;

  Vector operator-(const Vector& v) const;

  Vector operator*(float scalar) const;

  Vector& operator+=(const Vector&);

  bool operator==(const Vector& v) const;

  float dot(const Vector& v) const;

  float norm() const;

  Vector shortest_delta(const Vector& v, float x_max, float y_max) const;

  float distance(const Vector& v, float x_max_, float y_max_) const;
};

}  // namespace math
#endif