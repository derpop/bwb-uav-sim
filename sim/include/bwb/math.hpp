#pragma once
#include <cmath>
#include <algorithm>

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

        Mat3 inverse() const {
            double det = determinant();
            return Mat3(
                (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det,
                (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det,
                (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det,
                (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det,
                (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det,
                (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det,
                (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det,
                (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det,
                (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det
            );
        }
    };

    struct Euler {
        
        double phi, theta, psi; 
        // roll [rad], about body x, right wing down positive
        // pitch [rad], about body y, nose up positive
        // yaw [rad], about body z, right turn positive 
        explicit Euler(double phi_, double theta_ , double psi_ )
            : phi(phi_), theta(theta_), psi(psi_) {}

        Euler() : phi(0), theta(0), psi(0) {}
    };

    inline Mat3 euler_to_dcm(const Euler& euler) {

        const double cphi = std::cos(euler.phi),   sphi = std::sin(euler.phi);
        const double cth  = std::cos(euler.theta), sth  = std::sin(euler.theta);
        const double cpsi = std::cos(euler.psi),   spsi = std::sin(euler.psi);
        //Step 1 Yaw psi about z(heading)

        Mat3 R_psi = Mat3(
            cpsi, spsi, 0,
            -spsi,  cpsi, 0,
            0,     0,    1
        );

        Mat3 R_theta = Mat3(
            cth, 0, -sth,
            0,   1,  0,
            sth, 0,  cth
        );

        Mat3 R_phi = Mat3(
            1, 0,   0,
            0, cphi, sphi,
            0,-sphi, cphi
        );

        return R_psi.transpose() * R_theta.transpose() * R_phi.transpose();
    }

    // Unit Quaternion, scalar first e0 = cos theta /2
    struct Quat{
        double e0, e1, e2, e3;

        explicit Quat(double e0_, double e1_, double e2_, double e3_)
            : e0(e0_), e1(e1_), e2(e2_), e3(e3_) {}
            
        Quat() : e0(1), e1(0), e2(0), e3(0) {}

        double norm() const {
            return std::sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3);
        }
        //Zero quaternion gives NaNs
        Quat normalized() const {
            double n = norm();
            return Quat(e0 / n, e1 / n, e2 / n, e3 / n);
        }

        Quat operator*(double k) const {
            return Quat(e0 * k, e1 * k, e2 * k, e3 * k);
        }

    };

    inline Quat quat_multiply(const Quat& q1, const Quat& q2) {
        return Quat(
            q1.e0 * q2.e0 - q1.e1 * q2.e1 - q1.e2 * q2.e2 - q1.e3 * q2.e3,
            q1.e0 * q2.e1 + q1.e1 * q2.e0 + q1.e2 * q2.e3 - q1.e3 * q2.e2,
            q1.e0 * q2.e2 - q1.e1 * q2.e3 + q1.e2 * q2.e0 + q1.e3 * q2.e1,
            q1.e0 * q2.e3 + q1.e1 * q2.e2 - q1.e2 * q2.e1 + q1.e3 * q2.e0
        );
    }

    inline Quat euler_to_quat(const Euler& euler) {
        const double cphi = std::cos(euler.phi / 2),   sphi = std::sin(euler.phi / 2);
        const double cth  = std::cos(euler.theta / 2), sth  = std::sin(euler.theta / 2);
        const double cpsi = std::cos(euler.psi / 2),   spsi = std::sin(euler.psi / 2);

        return Quat(
            cphi * cth * cpsi + sphi * sth * spsi,
            sphi * cth * cpsi - cphi * sth * spsi,
            cphi * sth * cpsi + sphi * cth * spsi,
            cphi * cth * spsi - sphi * sth * cpsi
        );
    }

    inline Mat3 rot_body_to_ned(const Quat& q) {
        double e0 = q.e0, e1 = q.e1, e2 = q.e2, e3 = q.e3;
        return Mat3(
            1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e3 * e0), 2 * (e1 * e3 + e2 * e0),
            2 * (e1 * e2 + e3 * e0), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e1 * e0),
            2 * (e1 * e3 - e2 * e0), 2 * (e2 * e3 + e1 * e0), 1 - 2 * (e1 * e1 + e2 *(e2))
        );
    }
    
    inline Euler quat_to_euler(const Quat& q) {
        double e0 = q.e0, e1 = q.e1, e2 = q.e2, e3 = q.e3;
        double phi = std::atan2(2 * (e0 * e1 + e2 * e3), e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2);
        double theta = std::asin(std::clamp(2 * (e0 * e2 - e3 * e1), -1.0, 1.0));
        double psi = std::atan2(2 * (e0 * e3 + e1 * e2), e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3);
        return Euler(phi, theta, psi);
    }
}
