#include "bwb/rigid_body.hpp"

namespace bwb{
    MassProps::MassProps(double m_, double Jx_, double Jy_, double Jz_, double Jxz_){
        m = m_;
        Jx = Jx_;
        Jy = Jy_;
        Jz = Jz_;
        Jxz = Jxz_;
        G = Jx * Jz - Jxz * Jxz;
        G1 = (Jxz * ( Jx - Jy + Jz) )/ G;
        G2 = (Jz * ( Jz - Jy) + Jxz * Jxz) / G;
        G3 = Jz / G;
        G4 = Jxz / G;
        G5 = (Jz - Jx) / Jy;
        G6 = Jxz / Jy;
        G7 = ((Jx - Jy) * Jx + Jxz * Jxz) / G;
        G8 = Jx / G;
    }
    State::State(){
        pos_ned = Vec3();
        vel_body = Vec3();
        att = Quat();
        omega = Vec3();
    }
    State State::operator+(const State& other) const {
        State result;
        result.pos_ned = pos_ned + other.pos_ned;
        result.vel_body = vel_body + other.vel_body;
        result.att = att + other.att;
        result.omega = omega + other.omega;
        return result;
    }
    
    State State::operator-(const State& other) const {
        State result;
        result.pos_ned = pos_ned - other.pos_ned;
        result.vel_body = vel_body - other.vel_body;
        result.att = att - other.att;
        result.omega = omega - other.omega;
        return result;
    }
    
    State State::operator*(double scalar) const {
        State result;
        result.pos_ned = pos_ned * scalar;
        result.vel_body = vel_body * scalar;
        result.att = att * scalar;
        result.omega = omega * scalar;
        return result;
    }
    State operator*(double scalar, const State& state){
        return state * scalar;
    }
    State state_derivative(const State&x, const Vec3& force_body, 
            const Vec3& moment_body, const MassProps& mass){
        State dx;
        dx.pos_ned = rot_body_to_ned(x.att) * x.vel_body;
        dx.vel_body = (x.vel_body.crossProd(x.omega)) + force_body / mass.m;
        const Quat e_times_quat = quat_multiply(
            x.att, Quat(0, x.omega.x, x.omega.y, x.omega.z));
        dx.att = Quat(0.5 * e_times_quat.e0, 0.5 * e_times_quat.e1,
                      0.5 * e_times_quat.e2, 0.5 * e_times_quat.e3);
        const double p = x.omega.x ;
        const double q = x.omega.y ;
        const double r = x.omega.z ;
        const double l = moment_body.x;
        const double M_pitch = moment_body.y;
        const double n = moment_body.z;
        double p_dot = (mass.G1 * p * q) - (mass.G2 * q * r) + (mass.G3 * l) + (mass.G4 * n);
        double q_dot = (mass.G5 * p * r) - (mass.G6 * (p * p - r * r)) + (M_pitch / mass.Jy);
        double r_dot = (mass.G7 * p * q) - (mass.G1 * q * r) + (mass.G4 * l) + (mass.G8 * n);
        dx.omega = Vec3(p_dot, q_dot, r_dot);
        return dx;
    }

    State rk4_step(const State& x, const Vec3& force, const Vec3& moment,
         const MassProps& mass, double dt){
        return rk4_step(x, dt, [&](const State& s) { return state_derivative(s, force, moment, mass); });
    }
    // Perform a single Runge-Kutta 4th order step for the state given a derivative function.
    // I did it this way because my test calls could stay the same and i get the new functionality of being able to pass a custom derivative function.
    State rk4_step(const State& x, double dt, const std::function<State(const State&)>& deriv){
        State k1 = deriv(x);
        State k2 = deriv(x + 0.5 * dt * k1);
        State k3 = deriv(x + 0.5 * dt * k2);
        State k4 = deriv(x + dt * k3);
        State next = x + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
        next.att = next.att.normalized();
        return next;
    }
}