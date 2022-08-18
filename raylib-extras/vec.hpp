#ifndef _VEC_HPP_
#define _VEC_HPP_

#include "raylib.h"
#include <cmath> // std:sin, std::cos, std::sqrt

// Make this more generic using a union or std::variant?

namespace ai
{

struct Vector2 : public ::Vector2
{
  using self_t = Vector2;

  Vector2() = default;
  Vector2(const self_t&) = default;
  Vector2(self_t&&) = default;
  ~Vector2() = default;
  Vector2(float x, float y) : ::Vector2{x, y} {}

  self_t &operator=(const self_t&) = default;
  self_t &operator=(const float f) { x=f; y=f; return *this; }

  self_t operator+(const float f) const { return {x+f,y+f}; }
  self_t operator-(const float f) const { return {x-f,y-f}; }
  self_t operator*(const float f) const { return {x*f,y*f}; }
  friend self_t operator*(const float f, const self_t& r) { return r*f; }
  self_t operator/(const float f) const { return {x/f,y/f}; }

  self_t operator+(const self_t &r) const { return {x+r.x, y+r.y}; }
  self_t operator-(const self_t &r) const { return {x-r.x, y-r.y}; }

  self_t &operator+=(const self_t &r) { x+=r.x; y+=r.y; return *this; }
  self_t &operator/=(const self_t &r) { x/=r.x; y/=r.y; return *this; }

  self_t &operator*=(const float f) { x*=f; y*=f; return *this; }
  self_t &operator/=(const float f) { x/=f; y/=f; return *this; }

  float length() const { return std::sqrt(x*x+y*y); }
  void normalise() { (*this) /= (length() == 0 ? 1 : length()); }
};

struct Vector3 : public ::Vector3
{
  using self_t = Vector3;

  Vector3() = default;
  Vector3(const self_t&) = default;
  Vector3(self_t&&) = default;
  ~Vector3() = default;
  Vector3(float x, float y, float z) : ::Vector3{x, y, z} {}

  self_t &operator=(const self_t&) = default;
  self_t &operator=(const float f) { x=f; y=f; z=f; return *this; }

  self_t operator+(const float f) const { return {x+f,y+f,z+f}; }
  self_t operator-(const float f) const { return {x-f,y-f,z-f}; }
  self_t operator*(const float f) const { return {x*f,y*f,z*f}; }
  friend self_t operator*(const float f, const self_t& r) { return r*f; }
  self_t operator/(const float f) const { return {x/f,y/f,z/f}; }

  self_t operator+(const self_t &r) const { return {x+r.x, y+r.y, z+r.z}; }
  self_t operator-(const self_t &r) const { return {x-r.x, y-r.y, z-r.z}; }

  self_t &operator+=(const self_t &r) { x+=r.x; y+=r.y; z+=r.z; return *this; }
  self_t &operator/=(const self_t &r) { x/=r.x; y/=r.y; z/=r.z; return *this; }

  self_t &operator*=(const float f) { x*=f; y*=f; z*=f; return *this; }
  self_t &operator/=(const float f) { x/=f; y/=f; z/=f; return *this; }

  float length() const { return std::sqrt(x*x+y*y+z*z); }
  void normalise() { (*this) /= (length() == 0 ? 1 : length()); }
};

inline Vector3 asVector(const float orientation) {
  return {-std::sin(orientation), 0, std::cos(orientation)};
}

} // namespace ai

#endif // _VEC_HPP_
