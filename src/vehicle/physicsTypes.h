#pragma once

#include <array>
#include <cmath>
#include <cwchar>

#define _USE_MATH_DEFINES
#include <math.h>

class Angle {
    float value;

    static float normalize(double v);

   public:
    Angle(float v = 0.0f) : value(normalize(v)) {}
    float get() const;
    float getRadians();

    Angle operator+(float rhs) const;
    Angle operator-(float rhs) const;

    Angle operator+(const Angle& rhs) const;
    Angle operator-(const Angle& rhs) const;

    Angle& operator+=(float rhs);
    Angle& operator-=(float rhs);

    Angle& operator+=(const Angle& rhs);
    Angle& operator-=(const Angle& rhs);
};

template <typename T>
class Vec3 {
    T x;
    T y;
    T z;

   public:
    Vec3<T> operator+(Vec3<T> rhs);
    Vec3<T> operator-(Vec3<T> rhs);

    Vec3<T>& operator+=(Vec3<T> rhs);
    Vec3<T>& operator-=(Vec3<T> rhs);
};

template <>
class Vec3<float> {
    float x, y, z;

   public:
    Vec3(float length, Angle phi, Angle theta);
    Vec3<float> PolarTransform(float r = 0.0f, Angle phi = 0, Angle theta = 0);
};

using Vec3f = Vec3<float>;
using Torque = Vec3f;
using ForceVec = Vec3f;

template <typename T>
struct Positioned {
    T value;
    Vec3f position;

   public:
    Positioned<T> operator+(Positioned<T> rhs);
    Positioned<T> operator-(Positioned<T> rhs);

    Positioned<T>& operator+=(Positioned<T> rhs);
    Positioned<T>& operator-=(Positioned<T> rhs);
};

using MasiveObject = Positioned<float>;
using ForcefullObject = Positioned<ForceVec>;
