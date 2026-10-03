#pragma once
#include "bwb/math.hpp"
#include <functional>


namespace bwb{
    struct MassProps {
        double m, Jx, Jy, Jz, Jxz;
        double G, G1, G2, G3, G4, G5, G6, G7, G8;
        MassProps(double m_, double Jx_, double Jy_, double Jz_, double Jxz_);
    };
    struct State{
        Vec3 pos_ned;
        Vec3 vel_body;
        Quat att;
        Vec3 omega;
        State();
        State operator+(const State& other) const;
        State operator-(const State& other) const;
        State operator*(double scalar) const;
        friend State operator*(double scalar, const State& state);
    };
    State state_derivative(const State&x, const Vec3& force_body, 
            const Vec3& moment_body, const MassProps& mass);
    State rk4_step(const State& x, double dt, const std::function<State(const State&)>& deriv);
    State rk4_step(const State& x, const Vec3& force, const Vec3& moment,
         const MassProps& mass, double dt);
}
