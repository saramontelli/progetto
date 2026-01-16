// classe che rappresenta un vettore matematico bidimensionale
// quindi devo avere 2 omponenti, le operazioni matematiche nautarli.

#ifndef VECTOR_HPP  // se vector_h non è ancora stato definito allora definiscilo
#define VECTOR_HPP

namespace math {
class Vector {
 private:
  float x_;
  float y_;

 public:
  Vector();                  //(0,0)
  Vector(float x, float y);  //(x,y)

  float get_x() const;
  float get_y() const;

  void set_x(float newX);
  void set_y(float newY);

  // definisco le operazioni tra i vettori
  // somma: (x1,y1)+(x2,y2)
  Vector operator+(const Vector& v) const;
  // sottrazione: (x1,y1)-(x2,y2)
  Vector operator-(const Vector& v) const;
  // moltiplicazione per uno scalare: l*(a,b)
  Vector operator*(float scalar) const;
  // sommare un altro vettore modificando l'oggetto stesso
  Vector& operator+=(const Vector&);

  // confronto tra vettori
  bool operator==(const Vector& v) const;
  // prodotto scalare (x1;y1)(x2;y2)=x1x2+y1y2
  float dot(const Vector& v) const;
  // norma
  float norm() const;
  // distanza
  float distance(const Vector& v) const;
};

}  // namespace math
#endif