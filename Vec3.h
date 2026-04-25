#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <cstdio>

class Vec3{
private:
    static constexpr double EPSILON = 1e-6;
    double x;
    double y;
    double z;
public:

    void Print() const {
        printf("(%f, %f, %f)\n", x, y, z);
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

    Vec3 Normalize() const {
        double magnitude = Magnitude();
        if (magnitude < EPSILON) return Vec3(0.0, 0.0, 0.0);
        return Vec3(x / magnitude, y / magnitude, z / magnitude);
    }

    double Magnitude() const {
        return sqrt((x * x) + (y * y) + (z * z));
    }

    Vec3 AddVec3(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 SubtractVec3(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 MultiplyVec3(const double& s) const {
        return Vec3(x * s, y * s, z * s);
    }

    Vec3 CrossProduct(const Vec3& other) const {
        return Vec3(
            (y * other.z - z * other.y),
            (z * other.x - x * other.z),
            (x * other.y - y * other.x)
        );
    }

    double DotProduct(const Vec3& other) const {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    Vec3 operator+(const Vec3& other) const {
        return AddVec3(other);
    }

    Vec3 operator-(const Vec3& other) const {
        return SubtractVec3(other);
    }

    Vec3 operator*(const double& scalar) const {
        return MultiplyVec3(scalar);
    }

    Vec3() : x(0.0), y(0.0), z(0.0) {}

    Vec3(const double& x_, const double& y_, const double& z_) : x(x_), y(y_), z(z_) {}
};

#endif
