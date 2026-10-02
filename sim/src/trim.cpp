#include "bwb/trim.hpp"
#include <cmath>

namespace{
    bwb::State fromTrimInput(double alpha, double Va, double gamma){
        bwb::State x;
        x.vel_body = bwb::Vec3(Va* std::cos(alpha),0.0, Va* std::sin(alpha));
        x.att = bwb::euler_to_quat(bwb::Euler(0.0, alpha+gamma, 0.0));
        return x;
    }
    double elevFromAlpha(double alpha, const bwb::AeroParams& p){
        return -(p.C_m_0 + p.C_m_alpha * alpha) / p.C_m_de;
    }
}



namespace bwb {
    TrimResult trim(double Va, double gamma, const AeroParams& p, const MassProps& mass) {
        TrimResult result;
        double lo = -0.1;
        double hi = 0.3;
        double rLo = residual_of_alpha(lo, Va, gamma, p, mass);
        double rHi = residual_of_alpha(hi, Va, gamma, p, mass);
        if ((rLo > 0) == (rHi > 0)) {
            result.status = TrimStatus::no_bracket;
            return result;
        }
        while (hi - lo > 1e-12) {
            double mid = (lo + hi) / 2;
            double rMid = residual_of_alpha(mid, Va, gamma, p, mass);
            if ((rLo > 0) == (rMid > 0)) {
                lo = mid;
                rLo = rMid;
            } else {
                hi = mid;
            }
        }
        double alpha = (lo + hi) / 2;
        const State x = fromTrimInput(alpha, Va, gamma);
        Controls ctrl;
        ctrl.de =  elevFromAlpha(alpha, p);
        const Vec3 wind;
        const ForcesMoments forces = forces_moments(x, ctrl, wind, p, mass);
        if( forces.force.x > 0){
            result.status = TrimStatus::throttle_out_of_range;
            return result;
        }
        ctrl.dt = std::sqrt((-2 * forces.force.x)/(p.rho * p.S_prop * p.C_prop * p.k_motor * p.k_motor));
        if(ctrl.dt > 1){
            result.status = TrimStatus::throttle_out_of_range;
            return result;
        }
        result.status = TrimStatus::ok;
        result.alpha = alpha;
        result.ctrl = ctrl;
        result.x = x;
        return result;
    }

    double residual_of_alpha(double alpha, double Va, double gamma, const AeroParams& p, const MassProps& mass) {
        const State x = fromTrimInput(alpha, Va, gamma);
        Controls ctrl;
        ctrl.de =  elevFromAlpha(alpha, p);
        const Vec3 wind;
        //All other comps are zero by default
        const ForcesMoments forces = forces_moments(x, ctrl, wind, p, mass);
        return forces.force.z;
    }

}
