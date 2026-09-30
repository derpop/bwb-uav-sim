#pragma once
#include "bwb/math.hpp"


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
        
    };
    State state_derivative(const State&x, const Vec3& force_body, 
            const Vec3& moment_body, const MassProps& mass);
}
