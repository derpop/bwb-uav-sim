#pragma once
#include <cmath>

namespace bwb{
    struct Vec3{
    
        double x, y, z;
    
    
        explicit Vec3(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}    
    
        Vec3 operator+(const Vec3& other) const {
            return Vec3(x + other.x, y + other.y, z + other.z);
        }
    
        Vec3 operator-(const Vec3& other) const {
            return Vec3(x - other.x, y - other.y, z - other.z);
        }

        Vec3 operator-() const {
            return Vec3(-x, -y, -z);
        }
        
        friend Vec3 operator*(double scalar, const Vec3& v) {
            return v*scalar;
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

        Vec3 operator/(double scalar) const {
            return Vec3(x / scalar, y / scalar, z / scalar);
        }
    };
}
    
namespace bwb {
    struct Mat3{
        double m[3][3];

        Mat3() {
            m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
            m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
            m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
        }
        Mat3(double m00, double m01, double m02,
             double m10, double m11, double m12,
             double m20, double m21, double m22) {
            this->m[0][0] = m00; this->m[0][1] = m01; this->m[0][2] = m02;
            this->m[1][0] = m10; this->m[1][1] = m11; this->m[1][2] = m12;
            this->m[2][0] = m20; this->m[2][1] = m21;	this->m[2][2] =	m22;
        }
    

        Mat3 operator+(const Mat3& other) const {
            Mat3 result;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    result.m[i][j] = m[i][j] + other.m[i][j];
                }
            }
            return result;
        }
        Mat3 operator-(const Mat3& other) const {
            Mat3 result;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    result.m[i][j] = m[i][j] - other.m[i][j];
                }
            }
            return result;
        }
        Mat3 operator*(const Mat3& other) const {
            Mat3 result;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    result.m[i][j] = 0;
                    for (int k = 0; k < 3; ++k) {
                        result.m[i][j] += m[i][k] * other.m[k][j];
                    }
                }
            }
            return result;
        }
        Mat3 operator*(double scalar) const {
            Mat3 result;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    result.m[i][j] = m[i][j] * scalar;
                }
            }
            return result;
        }
        Mat3 transpose() const {
            Mat3 result;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    result.m[i][j] = m[j][i];
                }
            }
            return result;
        }

        double determinant() const {
            return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                 - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                 + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        }

        Vec3 operator*(const Vec3& other) const {
            return Vec3(
                m[0][0] * other.x + m[0][1] * other.y + m[0][2] * other.z,
                m[1][0] * other.x + m[1][1] * other.y + m[1][2] * other.z,
                m[2][0] * other.x + m[2][1] * other.y + m[2][2] * other.z
            );
        }
    };
}
