/*
*   LICENSE: zlib/libpng
*
*   raylib-cpp is licensed under an unmodified zlib/libpng license, which is an OSI-certified,
*   BSD-like license that allows static linking with closed source software:
*
*   Copyright (c) 2020 Rob Loach (@RobLoach)
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*/

#ifndef RAYLIB_CPP_INCLUDE_VECTOR2_HPP_
#define RAYLIB_CPP_INCLUDE_VECTOR2_HPP_

#ifndef RAYLIB_CPP_NO_MATH
#include <cmath>
#endif

#include "./raylib.hpp"
#include "./raymath.hpp"
#include "./raylib-cpp-utils.hpp"

namespace raylib {
class Vector2 : public ::Vector2 {
 public:
    Vector2(const ::Vector2& vec) {
        set(vec);
    }

    Vector2(float x, float y) : ::Vector2{x, y} {}
    Vector2(float x) : ::Vector2{x, 0} {}
    Vector2() : ::Vector2{0, 0} {}

    GETTERSETTER(float, X, x)
    GETTERSETTER(float, Y, y)

    Vector2& operator=(const ::Vector2& vector2) {
        set(vector2);
        return *this;
    }

    bool operator==(const ::Vector2& other) {
        return x == other.x
            && y == other.y;
    }

#ifndef RAYLIB_CPP_NO_MATH
    Vector2 Add(const ::Vector2& vector2) const {
        return Vector2Add(*this, vector2);
    }

    Vector2 operator+(const ::Vector2& vector2) {
        return Vector2Add(*this, vector2);
    }

    Vector2 Subtract(const ::Vector2& vector2) const {
        return Vector2Subtract(*this, vector2);
    }

    Vector2 operator-(const ::Vector2& vector2) {
        return Vector2Subtract(*this, vector2);
    }

    Vector2 Negate() const {
        return Vector2Negate(*this);
    }

    Vector2 operator-() {
        return Vector2Negate(*this);
    }

    Vector2 Multiply(const ::Vector2& vector2) const {
        return Vector2Multiply(*this, vector2);
    }

    Vector2 operator*(const ::Vector2& vector2) {
        return Vector2Multiply(*this, vector2);
    }

    Vector2 Scale(const float scale) const {
        return Vector2Scale(*this, scale);
    }

    Vector2 operator*(const float scale) {
        return Vector2Scale(*this, scale);
    }

    Vector2 Divide(const ::Vector2& vector2) const {
        return Vector2Divide(*this, vector2);
    }

    Vector2 operator/(const ::Vector2& vector2) {
        return Vector2Divide(*this, vector2);
    }

    Vector2& Divide(const float div) {
        x /= div;
        y /= div;

        return *this;
    }

    Vector2& operator/(const float div) {
        x /= div;
        y /= div;

        return *this;
    }

    Vector2& operator+=(const ::Vector2& vector2) {
        set(Vector2Add(*this, vector2));

        return *this;
    }

    Vector2& operator-=(const ::Vector2& vector2) {
        set(Vector2Subtract(*this, vector2));

        return *this;
    }


    Vector2& operator*=(const ::Vector2& vector2) {
        set(Vector2Multiply(*this, vector2));

        return *this;
    }

    Vector2& operator*=(const float scale) {
        set(Vector2Scale(*this, scale));

        return *this;
    }

    Vector2& operator/=(const ::Vector2& vector2) {
        set(Vector2Divide(*this, vector2));

        return *this;
    }

    Vector2& operator/=(const float div) {
        this->x /= div;
        this->y /= div;

        return *this;
    }

    /**
     * Calculate vector length
     */
    float Length() const {
        return Vector2Length(*this);
    }

    /**
     * Calculate vector square length
     */
    float LengthSqr() const {
        return Vector2LengthSqr(*this);
    }

    /**
     * Normalize provided vector
     */
    Vector2 Normalize() const {
        return Vector2Normalize(*this);
    }

    /**
     * Calculate two vectors dot product
     */
    float DotProduct(const ::Vector2& vector2) const {
        return Vector2DotProduct(*this, vector2);
    }

    /**
     * Calculate angle from two vectors in X-axis
     */
    float Angle(const ::Vector2& vector2) const {
        return Vector2Angle(*this, vector2);
    }

    /**
     * Calculate distance between two vectors
     */
    float Distance(const ::Vector2& vector2) const {
        return Vector2Distance(*this, vector2);
    }

    /**
     * Calculate linear interpolation between two vectors
     */
    Vector2 Lerp(const ::Vector2& vector2, const float amount) const {
        return Vector2Lerp(*this, vector2, amount);
    }

    /**
     * Calculate reflected vector to normal
     */
    Vector2 Reflect(const ::Vector2& normal) const {
        return Vector2Reflect(*this, normal);
    }

    /**
     * Rotate Vector by float in Degrees
     */
    Vector2 Rotate(float degrees) const {
        return Vector2Rotate(*this, degrees);
    }

    /**
     * Move Vector towards target
     */
    Vector2 MoveTowards(const ::Vector2& target, float maxDistance) const {
        return Vector2MoveTowards(*this, target, maxDistance);
    }

    /**
     * Vector with components value 0.0f
     */
    static Vector2 Zero() {
        return Vector2Zero();
    }

    /**
     * Vector with components value 1.0f
     */
    static Vector2 One() {
        return Vector2One();
    }
#endif

    inline Vector2& DrawPixel(::Color color) {
        ::DrawPixelV(*this, color);
        return *this;
    }

    inline Vector2& DrawLine(::Vector2 endPos, ::Color color) {
        ::DrawLineV(*this, endPos, color);
        return *this;
    }

    inline Vector2& DrawLine(::Vector2 endPos, float thick, ::Color color) {
        ::DrawLineEx(*this, endPos, thick, color);
        return *this;
    }

    inline Vector2& DrawLineBezier(::Vector2 endPos, float thick, ::Color color) {
        ::DrawLineBezier(*this, endPos, thick, color);
        return *this;
    }

    /**
     * Draw line using quadratic bezier curves with a control point
     *
     * TODO(RobLoach): Add this one this commit makes it in: https://github.com/raysan5/raylib/commit/521ed1cef0c0dbc752cb79f70c21bb4fc3bf70d4
     */
    //inline Vector2& DrawLineBezierQuad(::Vector2 endPos, ::Vector2 controlPos, float thick, ::Color color) {
    //    ::DrawLineBezierQuad(*this, endPos, controlPos, thick, color);
    //    return *this;
    //}

    /**
     * Draw a color-filled circle (Vector version)
     */
    inline Vector2& DrawCircle(float radius, ::Color color) {
        ::DrawCircleV(*this, radius, color);
        return *this;
    }

    inline Vector2& DrawRectangle(::Vector2 size, ::Color color) {
        ::DrawRectangleV(*this, size, color);
        return *this;
    }

    inline Vector2& DrawPoly(int sides, float radius, float rotation, ::Color color) {
        ::DrawPoly(*this, sides, radius, rotation, color);
        return *this;
    }

    /**
     * Check collision between two circles
     */
    inline bool CheckCollisionCircle(float radius1, ::Vector2 center2, float radius2) const {
        return ::CheckCollisionCircles(*this, radius1, center2, radius2);
    }

    /**
     * Check collision between circle and rectangle
     */
    inline bool CheckCollisionCircle(float radius, ::Rectangle rec) const {
        return ::CheckCollisionCircleRec(*this, radius, rec);
    }

    /**
     * Check if point is inside rectangle
     */
    inline bool CheckCollision(::Rectangle rec) const {
        return ::CheckCollisionPointRec(*this, rec);
    }

    /**
     * Check if point is inside circle
     */
    inline bool CheckCollision(::Vector2 center, float radius) const {
        return ::CheckCollisionPointCircle(*this, center, radius);
    }

    /**
     * Check if point is inside a triangle
     */
    inline bool CheckCollision(::Vector2 p1, ::Vector2 p2, ::Vector2 p3) const {
        return ::CheckCollisionPointTriangle(*this, p1, p2, p3);
    }

    /**
     * Check the collision between two lines defined by two points each, returns collision point by reference
     */
    inline bool CheckCollisionLines(
            ::Vector2 endPos1,
            ::Vector2 startPos2, ::Vector2 endPos2,
            ::Vector2 *collisionPoint) const {
        return ::CheckCollisionLines(*this, endPos1, startPos2, endPos2, collisionPoint);
    }

 protected:
    inline void set(const ::Vector2& vec) {
        x = vec.x;
        y = vec.y;
    }
};

}  // namespace raylib

#endif  // RAYLIB_CPP_INCLUDE_VECTOR2_HPP_
