// Mass properties for the aircraft the sim knows about.
//
// Body axes follow Beard & McLain: x forward (out the nose), y right wing,
// z down, origin at the CG. Jxz = integral of (x z) dm in those axes.
#pragma once

namespace bwb {

struct MassProps {
    double mass;  // kg
    double Jx;    // kg m^2, about body x (roll)
    double Jy;    // kg m^2, about body y (pitch)
    double Jz;    // kg m^2, about body z (yaw)
    double Jxz;   // kg m^2, product of inertia (B&M sign convention)
};

// Aerosonde UAV, the B&M verification aircraft.
// Values from the book's companion code (mavsim_python, aerosonde_parameters.py).
// The 1st-edition Appendix E table lists mass 13.5 kg; the companion code uses
// 11.0 kg. Check against your copy and keep whichever the reference run uses.
inline constexpr MassProps kAerosonde{11.0, 0.8244, 1.135, 1.759, 0.1204};

// BWB, mass/inertia v2 (brief Rev 0.6, phase3/inertia_v2.md), preliminary.
// The inertia script works in geometry axes (x aft, z down). Flipping x to point
// forward flips the sign of Jxz, so its -0.005 becomes +0.005 here.
// TODO(mass v3): roll in the Rev 0.8 motor (+28 g aft) and the battery move to
// x ~= 0.418 m before using these for anything beyond smoke tests.
inline constexpr MassProps kBwbV2{2.4, 0.180, 0.098, 0.273, 0.005};

}  // namespace bwb
