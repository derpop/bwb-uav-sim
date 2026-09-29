#pragma once
#include <cmath>

namespace bwb{
    struct Vec3{
    
        double x, y, z;
    
    
        explicit Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    
        Vec3 operator+(const Vec3& other) const {
            return Vec3(x + other.x, y + other.y, z + other.z);
        }
    
        Vec3 operator-(const Vec3& other) const {
            return Vec3(x - other.x, y - other.y, z - other.z);
        }
    
        Vec3 operator*(double scalar) const {
            return Vec3(x * scalar, y * scalar, z * scalar);
        }
    
        Vec3 crossProd(const Vec3& other) const {
    
            return Vec3(
    
                y * other.z - z * other.y,
    
                z * other.x - x * other.z,
    
                x * other.y - y * other.x
    
            );
    
        }
    
        double dotProd(const Vec3& other) const {
            return x * other.x + y * other.y + z * other.z;
        }
    
        double magnitude() const {
            return std::sqrt(x * x + y * y + z * z);
        }
    
        Vec3 normalize() const {
            double mag = magnitude();
            if (mag == 0) {
    
                return Vec3(0, 0, 0);
    
            }
            return Vec3(x / mag, y / mag, z / mag);
    
        }
    };
} // namespace bwb3

