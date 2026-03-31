#include <cmath>
#include <cstdio>

class Vec3{
private:
    float x;
    float y;
    float z;
public:

    void print() const {
        printf("(%f, %f, %f)\n", x, y, z);
    }

    Vec3 Normalize() const {
        float magnitude = Magnitude();
        return Vec3((x / magnitude), (y / magnitude), (z / magnitude));
    }

    float Magnitude() const {
        return sqrt((x * x) + (y * y) + (z * z));
    }

    Vec3 AddVec3(const Vec3& other) const {
        return Vec3((x + other.x), (y + other.y), (z + other.z));
    }

    Vec3 SubtractVec3(const Vec3& other) const {
        return Vec3((x - other.x), (y - other.y), (z - other.z));
    }

    Vec3 MultiplyVec3(const float& s) const {
        return Vec3(x * s, y * s, z * s);
    }

    Vec3 CrossProduct(const Vec3& other) const {
        return Vec3((y * other.z - z * other.y), (z * other.x - x * other.z), (x * other.y - y * other.x));
    }

    float DotProduct(const Vec3& other) const {
        return ((x * other.x) + (y * other.y) + (z * other.z));
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

    Vec3 operator*(const float& scalar) const {
        return MultiplyVec3(scalar);
    }

    Vec3(const float& x_, const float& y_, const float& z_) : x(x_), y (y_), z(z_){}
};