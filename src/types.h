#pragma once

#include <array>
#include <cmath>
#include <cwchar>

#define _USE_MATH_DEFINES
#include <math.h>

class Angle {
    float value;

    static float normalize(double v) {
        v = std::fmod(v, 360.0);
        return v < 0 ? v + 360.0 : v;
    }

   public:
    Angle(float v = 0) : value(normalize(v)) {}
    float get() const { return value; }

    Angle operator+(float rhs) const { return Angle(value + rhs); }
    Angle operator-(float rhs) const { return Angle(value - rhs); }

    Angle& operator+=(float rhs) {
        value = normalize(value + rhs);
        return *this;
    }
    Angle& operator-=(float rhs) {
        value = normalize(value - rhs);
        return *this;
    }

    float getRadians() { return value / 180.0f * M_PI; }
};

struct PolarVec3 {
    float amplitude;
    Angle alfa;
    Angle ro;
};

template <typename T>
struct Vec3 {
    T x;
    T y;
    T z;

    // Vec3<T> operator+(Vec3<T> rhs);
    // Vec3<T> operator-(Vec3<T> rhs);

    // Vec3<T>& operator+=(Vec3<T> rhs);
    // Vec3<T>& operator-=(Vec3<T> rhs);
};

template <>
struct Vec3<float> {
    float x, y, z;

    // Vec3(float length, Angle phi, Angle theta);
    // Vec3<float> PolarTransform(float r = 0.0f, Angle phi = 0, Angle theta = 0);
};

using Vec3f = Vec3<float>;

template <typename T>
struct Positioned {
    T value;
    Vec3f position;

    // Positioned<T> operator+(Positioned<T> rhs);
    // Positioned<T> operator-(Positioned<T> rhs);

    // Positioned<T>& operator+=(Positioned<T> rhs);
    // Positioned<T>& operator-=(Positioned<T> rhs);
};

using Mass = Positioned<float>;
using Force = Positioned<Vec3f>;
using Torque = Vec3f;

class MassiveObject {
   protected:
    Mass mass;
   public:
    Mass getMass() { return mass; }
};

class ForcefullObject {
   protected:
    Force force;
   public:
    Force getForce() { return force; }
};

class TorquedObject {
   protected:
    Torque torque;
   public:
    Torque getTorque() { return torque; }
};
