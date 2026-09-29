// Small fixed-size linear algebra for the 6-DOF sim.
//
// Deliberately dependency-free: the state is 13 numbers and the frames are 3x3,
// so a few hundred lines of plain C++ are easier to read and verify than a
// general matrix library. Swap in Eigen later if the GNC code needs it.
#pragma once

#include <array>
#include <cmath>

namespace bwb {

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

    constexpr double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double norm() const { return std::sqrt(dot(*this)); }
};

constexpr Vec3 operator*(double s, const Vec3& v) { return v * s; }

// Row-major 3x3 matrix.
struct Mat3 {
    std::array<std::array<double, 3>, 3> m{};

    constexpr Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    constexpr Mat3 transpose() const {
        Mat3 t;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) t.m[i][j] = m[j][i];
        return t;
    }
};

// Unit quaternion e = e0 + e1 i + e2 j + e3 k, scalar first (B&M Appendix B).
// Attitude of the body frame relative to the inertial (NED) frame.
struct Quat {
    double e0 = 1.0, e1 = 0.0, e2 = 0.0, e3 = 0.0;

    double norm() const { return std::sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3); }
    Quat normalized() const {
        const double n = norm();
        return {e0 / n, e1 / n, e2 / n, e3 / n};
    }
};

// Rotation matrix body -> inertial (NED), R_b^i in B&M notation.
inline Mat3 rot_body_to_ned(const Quat& q) {
    const double e0 = q.e0, e1 = q.e1, e2 = q.e2, e3 = q.e3;
    Mat3 r;
    r.m = {{{e1 * e1 + e0 * e0 - e2 * e2 - e3 * e3, 2 * (e1 * e2 - e3 * e0), 2 * (e1 * e3 + e2 * e0)},
            {2 * (e1 * e2 + e3 * e0), e2 * e2 + e0 * e0 - e1 * e1 - e3 * e3, 2 * (e2 * e3 - e1 * e0)},
            {2 * (e1 * e3 - e2 * e0), 2 * (e2 * e3 + e1 * e0), e3 * e3 + e0 * e0 - e1 * e1 - e2 * e2}}};
    return r;
}

// 3-2-1 (yaw psi, pitch theta, roll phi) Euler angles, radians.
struct Euler {
    double phi = 0.0, theta = 0.0, psi = 0.0;
};

// B&M App. B
inline Quat euler_to_quat(const Euler& a) {
    const double cp = std::cos(a.phi / 2), sp = std::sin(a.phi / 2);
    const double ct = std::cos(a.theta / 2), st = std::sin(a.theta / 2);
    const double cs = std::cos(a.psi / 2), ss = std::sin(a.psi / 2);
    return {cs * ct * cp + ss * st * sp,
            cs * ct * sp - ss * st * cp,
            cs * st * cp + ss * ct * sp,
            ss * ct * cp - cs * st * sp};
}

// B&M App. B
inline Euler quat_to_euler(const Quat& q) {
    const double e0 = q.e0, e1 = q.e1, e2 = q.e2, e3 = q.e3;
    double s = 2 * (e0 * e2 - e1 * e3);
    s = s > 1.0 ? 1.0 : (s < -1.0 ? -1.0 : s);  // guard asin at +/-90 deg pitch
    return {std::atan2(2 * (e0 * e1 + e2 * e3), e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2),
            std::asin(s),
            std::atan2(2 * (e0 * e3 + e1 * e2), e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3)};
}

}  // namespace bwb
