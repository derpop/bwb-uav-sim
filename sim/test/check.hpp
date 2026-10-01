#pragma once
#include "bwb/rigid_body.hpp"
#include <cmath>
#include <string>
#include <iostream>

inline int failures = 0; // Use inline to avoid multiple definition errors when including this header in multiple translation units

inline void check_near(double got, double want, double tol, const std::string& name) {
    if (!(std::abs(got - want) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cerr << "got: " << got << std::endl;
        std::cerr << "want: " << want << std::endl;
        failures++;
    }
}

inline void check_near(bwb::Vec3 got, bwb::Vec3 want, double tol, const std::string& name) {
    if (!(std::abs(got.x - want.x) <= tol &&  std::abs(got.y - want.y) <= tol &&  std::abs(got.z - want.z) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cerr << "got: " << got.x << ", " << got.y << ", " << got.z << std::endl;
        std::cerr << "want: " << want.x << ", " << want.y << ", " << want.z << std::endl;
        failures++;
    }
}

inline void check_near(bwb::Mat3 got, bwb::Mat3 want, double tol, const std::string& name) {
    bool ok = true;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!(std::abs(got.m[i][j] - want.m[i][j]) <= tol)) {
                ok = false;
                std::cerr << "Check failed: " << name << std::endl;
                std::cerr << "got: " << got.m[i][j] << ", want: " << want.m[i][j] << std::endl;
            }
        }
    }
    if (!ok) {
        failures++;
    }
}

inline void check_near(bwb::Euler got, bwb::Euler want, double tol, const std::string& name) {
    if (!(std::abs(got.phi - want.phi) <= tol && std::abs(got.theta - want.theta) <= tol &&  std::abs(got.psi - want.psi) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cerr << "got: " << got.phi << ", " << got.theta << ", " << got.psi << std::endl;
        std::cerr << "want: " << want.phi << ", " << want.theta << ", " << want.psi << std::endl;
        failures++;
    }
}

inline void check_near(bwb::Quat got, bwb::Quat want, double tol, const std::string& name) {
    if (!(std::abs(got.e0 - want.e0) <= tol &&  std::abs(got.e1 - want.e1) <= tol &&  std::abs(got.e2 - want.e2) <= tol && std::abs(got.e3 - want.e3) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cerr << "got: " << got.e0 << ", " << got.e1 << ", " << got.e2 << ", " << got.e3 << std::endl;
        std::cerr << "want: " << want.e0 << ", " << want.e1 << ", " << want.e2 << ", " << want.e3 << std::endl;
        failures++;
    }
}
inline bwb::State runRk4(bwb::State start, bwb::Vec3 force, bwb::Vec3 moment, double dt, bwb::MassProps mass_props, int steps) {
    bwb::State x = start;
    for (int i = 0; i < steps; i++) {
        x = bwb::rk4_step(x, force, moment, mass_props, dt);
    }
    return x;
}