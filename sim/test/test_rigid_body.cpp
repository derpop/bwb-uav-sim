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

void check_near(bwb::Mat3 got, bwb::Mat3 want, double tol, const std::string& name) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (std::abs(got.m[i][j] - want.m[i][j]) > tol) {
                std::cerr << "Check failed: " << name << std::endl;
                std::cout << "got: " << got.m[i][j] << ", want: " << want.m[i][j] << std::endl;
                failures++;
            }
        }
    }
}


int main() {
    //Vector Checks
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
    check_near(bwb::Vec3(1,0,0) * 2, bwb::Vec3(2,0,0), 1e-6, "scalar multiplication test");
    check_near(bwb::Vec3(1,0,0) / 2.0, bwb::Vec3(0.5,0,0), 1e-6, "scalar division test");
    check_near(bwb::Vec3(1,2,3).dotProd(bwb::Vec3(4,-5,6)), 12.0, 1e-6, "dotProd test");
    //Matrix Checks
    bwb::Mat3 identity = bwb::Mat3();
    check_near(identity, bwb::Mat3(1,0,0, 0,1,0, 0,0,1), 1e-6, "identity matrix test");
    bwb::Mat3 testing = bwb::Mat3(1,2,3, 4,5,6, 7,8,9);
    check_near(testing, bwb::Mat3(1,2,3, 4,5,6, 7,8,9), 1e-6, "testing matrix test");
    check_near(testing.transpose(), bwb::Mat3(1,4,7, 2,5,8, 3,6,9), 1e-6, "testing matrix transpose test");
    check_near(testing * identity, testing, 1e-6, "testing matrix multiplication with identity test");
    check_near(identity * testing, testing, 1e-6, "testing matrix multiplication with identity test (reverse)");
    check_near(testing * testing, bwb::Mat3(30,36,42, 66,81,96, 102,126,150), 1e-6, "testing matrix multiplication with itself test");
    bwb::Vec3 vector = bwb::Vec3(1,2,3);
    check_near(testing * vector, bwb::Vec3(14, 32, 50), 1e-6, "testing matrix-vector multiplication test");
    bwb::Mat3 testing2 = bwb::Mat3(9,8,7, 6,5,4, 3,2,1);
    check_near(testing2.transpose().transpose(), testing2, 1e-6, "testing2 matrix double transpose test");
    check_near((testing * testing2).transpose(), testing2.transpose() * testing.transpose(), 1e-6, "testing matrix multiplication transpose test");
    check_near(identity.determinant(), 1.0, 1e-6, "identity matrix determinant test");
    check_near(testing.determinant(), 0.0, 1e-6, "testing matrix determinant test");
    check_near(bwb::Mat3(1,2,3, 4,5,6, 7,8,10).determinant(), -3.0, 1e-6, "testing matrix determinant test nonsingular");
    // Euler to DCM test
    check_near(bwb::euler_to_dcm(bwb::Euler(0,0,M_PI/2)) * bwb::Vec3(1,0,0), bwb::Vec3(0, 1 , 0),  1e-6, "Nose East");
    check_near(bwb::euler_to_dcm(bwb::Euler(0,M_PI/6,0)) * bwb::Vec3(1,0,0), bwb::Vec3(cos(M_PI/6), 0 , -0.5),  1e-6, "Nose Up");
    check_near(bwb::euler_to_dcm(bwb::Euler(M_PI/2,0,0)) * bwb::Vec3(0,1,0), bwb::Vec3(0, 0 , 1),  1e-6, "Right Wing Down");
    bwb::Mat3 dcm = bwb::euler_to_dcm(bwb::Euler(0.3,-0.4,2.0));
    check_near(dcm * dcm.transpose(), bwb::Mat3(), 1e-6, "DCM orthogonality test");
    check_near(dcm.determinant(), 1.0, 1e-6, "DCM determinant test");
    std::cout << "Total failures: " << failures << std::endl;
    return failures == 0 ? 0 : 1;
}


