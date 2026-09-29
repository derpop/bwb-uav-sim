#include <iostream>
#include <cmath>
#include <string>
#include "bwb/math.hpp"

int failures = 0;

void check_near(double got, double want, double tol, const std::string& name) {
    if (!(std::abs(got - want) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cout << "got: " << got << std::endl;
        std::cout << "want: " << want << std::endl;
        failures++;
    }
}

void check_near(bwb::Vec3 got, bwb::Vec3 want, double tol, const std::string& name) {
    if (!(std::abs(got.x - want.x) <= tol &&  std::abs(got.y - want.y) <= tol &&  std::abs(got.z - want.z) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cout << "got: " << got.x << ", " << got.y << ", " << got.z << std::endl;
        std::cout << "want: " << want.x << ", " << want.y << ", " << want.z << std::endl;
        failures++;
    }
}

void check_near(bwb::Mat3 got, bwb::Mat3 want, double tol, const std::string& name) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!(std::abs(got.m[i][j] - want.m[i][j]) <= tol)) {
                std::cerr << "Check failed: " << name << std::endl;
                std::cout << "got: " << got.m[i][j] << ", want: " << want.m[i][j] << std::endl;
                failures++;
            }
        }
    }
}

void check_near(bwb::Euler got, bwb::Euler want, double tol, const std::string& name) {
    if (!(std::abs(got.phi - want.phi) <= tol && std::abs(got.theta - want.theta) <= tol &&  std::abs(got.psi - want.psi) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cout << "got: " << got.phi << ", " << got.theta << ", " << got.psi << std::endl;
        std::cout << "want: " << want.phi << ", " << want.theta << ", " << want.psi << std::endl;
        failures++;
    }
}

void check_near(bwb::Quat got, bwb::Quat want, double tol, const std::string& name) {
    if (!(std::abs(got.e0 - want.e0) <= tol &&  std::abs(got.e1 - want.e1) <= tol &&  std::abs(got.e2 - want.e2) <= tol && std::abs(got.e3 - want.e3) <= tol)) {
        std::cerr << "Check failed: " << name << std::endl;
        std::cout << "got: " << got.e0 << ", " << got.e1 << ", " << got.e2 << ", " << got.e3 << std::endl;
        std::cout << "want: " << want.e0 << ", " << want.e1 << ", " << want.e2 << ", " << want.e3 << std::endl;
        failures++;
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
    // Quaternion tests
    check_near(bwb::Quat(), bwb::Quat(1,0,0,0), 1e-6, "identity quaternion test");
    check_near(bwb::Quat(1,2,3,4).norm(), std::sqrt(30), 1e-6, "normal of quaternion test");
    check_near(bwb::Quat(1,2,3,4).normalized(), bwb::Quat(1/std::sqrt(30), 2/std::sqrt(30), 3/std::sqrt(30), 4/std::sqrt(30)), 1e-6, "normalized quaternion test (explicit)");
    bwb::Quat quat = bwb::Quat(1,2,3,4).normalized();
    check_near(quat.norm(), 1, 1e-6, "normalized quaternion norm test");
    check_near(bwb::quat_multiply(bwb::Quat(), quat), quat, 1e-6, "quaternion multiplication with identity test (reverse)");
    check_near(bwb::quat_multiply(quat, bwb::Quat()), quat, 1e-6, "quaternion multiplication with identity test");
    check_near(bwb::quat_multiply(bwb::Quat(0,1,0,0), bwb::Quat(0,0,1,0)), bwb::Quat(0,0,0,1), 1e-6, "quaternion multiplication test ij = k");
    check_near(bwb::quat_multiply(bwb::Quat(0,0,1,0), bwb::Quat(0,1,0,0)), bwb::Quat(0,0,0,-1), 1e-6, "quaternion multiplication test ji = -k");
    check_near(bwb::quat_multiply(bwb::Quat(0,1,13,12), bwb::Quat(12,13,1,42)).normalized(),bwb::quat_multiply(bwb::Quat(0,1,13,12).normalized(), bwb::Quat(12,13,1,42).normalized()), 1e-6, "quaternion multiplication test with normalization");
    //Euler to Quaternion test
    check_near(bwb::euler_to_quat(bwb::Euler(0,0,M_PI/2)), bwb::Quat(std::cos(M_PI/4),0,0,std::sin(M_PI/4)), 1e-6, "Euler to Quaternion test Yaw 90");
    check_near(bwb::euler_to_quat(bwb::Euler(M_PI/2,0,0)), bwb::Quat(std::cos(M_PI/4),std::sin(M_PI/4),0,0), 1e-6, "Euler to Quaternion test Roll 90");
    check_near(bwb::euler_to_quat(bwb::Euler(0.3,-0.4,2.0)).norm(), 1, 1e-6, "Unit Length Property");
    const double phi = 0.3, theta = -0.4, psi = 2.0;
    bwb::Quat q_psi  (std::cos(psi/2),   0, 0, std::sin(psi/2));    // yaw about z
    bwb::Quat q_theta(std::cos(theta/2), 0, std::sin(theta/2), 0);  // pitch about y
    bwb::Quat q_phi  (std::cos(phi/2),   std::sin(phi/2), 0, 0);    // roll about x
    bwb::Quat expected = bwb::quat_multiply(bwb::quat_multiply(q_psi, q_theta), q_phi);
    check_near(bwb::euler_to_quat(bwb::Euler(phi, theta, psi)), expected, 1e-12, "Euler to Quaternion test with individual rotations");
    // Total Check
    check_near(bwb::rot_body_to_ned(bwb::euler_to_quat(bwb::Euler(phi, theta, psi))), bwb::euler_to_dcm(bwb::Euler(phi, theta, psi)), 1e-12, "Total Check");
    check_near(bwb::rot_body_to_ned(bwb::Quat(1,2,3,4)), bwb::rot_body_to_ned(bwb::Quat(-1,-2,-3,-4)), 1e-12, "Total Check with opposite quaternion");
    check_near(bwb::euler_to_quat(bwb::Euler(0,0,2*M_PI)), bwb::Quat(-1,0,0,0), 1e-12, "Euler to Quaternion test with 360 degree yaw");
    //Round Trip
    bwb::Euler e = bwb::Euler(0.3, -0.4, 2.0);
    bwb::Quat q = bwb::euler_to_quat(e);
    bwb::Euler e_round_trip = bwb::quat_to_euler(q);
    check_near(e_round_trip, e, 1e-12, "Round Trip Euler to Quaternion to Euler");
    e = bwb::Euler(0, M_PI/2, 0.4);
    q = bwb::euler_to_quat(e);
    e_round_trip = bwb::quat_to_euler(q);
    check_near(e_round_trip.theta, M_PI/2, 1e-12, "Round Trip Euler to Quaternion to Euler with 90 degree pitch theta");
    if(!std::isfinite(e_round_trip.phi)){
        std::cerr << "phi is not finite" << std::endl;
        failures++;
    }
    if(!std::isfinite(e_round_trip.psi)){
        std::cerr << "psi is not finite" << std::endl;
        failures++;
    }    
    std::cout << "Total failures: " << failures << std::endl;
    return failures == 0 ? 0 : 1;
}


