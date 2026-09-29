#include <iostream>
#include <cmath>
#include <string>
#include "bwb/math.hpp"

int failures = 0;

void check_near(double got, double want, double tol, const std::string& name) {
    if (std::abs(got - want) > tol) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cout << "got: " << got << std::endl;
        std::cout << "want: " << want << std::endl;
        failures++;
    }
}

void check_near(bwb::Vec3 got, bwb::Vec3 want, double tol, const std::string& name) {
    if (std::abs(got.x - want.x) > tol || std::abs(got.y - want.y) > tol || std::abs(got.z - want.z) > tol) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cout << "got: " << got.x << ", " << got.y << ", " << got.z << std::endl;
        std::cout << "want: " << want.x << ", " << want.y << ", " << want.z << std::endl;
        failures++;
    }
}

int main() {
    check_near(bwb::Vec3(1, 0, 0).crossProd(bwb::Vec3(0, 1, 0)), bwb::Vec3(0, 0, 1), 1e-6, "crossProd test 1");
    check_near(bwb::Vec3(0, 1, 0).crossProd(bwb::Vec3(1, 0, 0)), bwb::Vec3(0, 0, -1), 1e-6, "crossProd test 2");
    check_near(bwb::Vec3(3,4,5).magnitude(), std::sqrt(50), 1e-6, "magnitude test");
    check_near(bwb::Vec3(1,2,3).crossProd(bwb::Vec3(4,5,6)), bwb::Vec3(-3, 6, -3), 1e-6, "crossProd test 3");
    check_near(bwb::Vec3(1,0,0).normalize(), bwb::Vec3(1,0,0), 1e-6, "normalize test 1");
    check_near(bwb::Vec3(0,1,0).normalize(), bwb::Vec3(0,1,0), 1e-6, "normalize test 2");
    check_near(bwb::Vec3(0,0,1).normalize(), bwb::Vec3(0,0,1), 1e-6, "normalize test 3");
    check_near(bwb::Vec3(1,0,0).normalize(), bwb::Vec3(1,0,0), 1e-6, "normalize test 4");
    check_near(bwb::Vec3(1,1,1).normalize(), bwb::Vec3(1/std::sqrt(3), 1/std::sqrt(3), 1/std::sqrt(3)), 1e-6, "normalize test 5");
    check_near(-bwb::Vec3(1,0,0), bwb::Vec3(-1,0,0), 1e-6, "negation test");
    check_near(2.0 * bwb::Vec3(1,0,0), bwb::Vec3(2,0,0), 1e-6, "scalar multiplication test");
    check_near(bwb::Vec3(1,0,0) / 2.0, bwb::Vec3(0.5,0,0), 1e-6, "scalar division test");
    std::cout << "Total failures: " << failures << std::endl;
    return failures == 0 ? 0 : 1;
}


