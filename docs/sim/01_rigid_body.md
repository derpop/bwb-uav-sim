# 01: Rigid-body equations of motion

The first layer of the sim is pure rigid-body mechanics: given the total force and moment on the aircraft, how does it move? Everything aerodynamic comes later and plugs in as a force/moment input. Reference: Beard & McLain (B&M) ch. 2–3 and Appendix B.

## Frames

- **Inertial (NED):** flat, non-rotating Earth. x north, y east, z down. Fine for a small UAV over a few km; Earth rotation and curvature are far below the aero model's uncertainty.
- **Body:** origin at the CG, x out the nose, y out the right wing, z down.

The BWB's aero and mass data were built in *geometry* axes (flow5 / inertia script: x aft, z up or down). Converting to body axes flips signs; keep every conversion in one place and test it.

## State (13)

| Symbol | Meaning | Frame | Unit |
|---|---|---|---|
| p_n, p_e, p_d | CG position | NED | m |
| u, v, w | CG velocity | body | m/s |
| e0, e1, e2, e3 | attitude quaternion | body wrt NED | – |
| p, q, r | angular rate | body | rad/s |

## Equations

**Translational kinematics.** Velocity is stored in body axes, so rotate it into NED:

  ṗ_ned = R_b^i(e) · [u, v, w]ᵀ

**Translational dynamics.** Newton's second law, m·dv/dt = f, holds in the inertial frame. Written in the rotating body frame the derivative picks up a transport term (the Coriolis theorem, B&M ch. 3):

  [u̇, v̇, ẇ]ᵀ = [r v − q w,  p w − r u,  q u − p v]ᵀ + f / m

**Rotational kinematics.** For a quaternion, B&M App. B:

  ė = ½ · Ω(p, q, r) · e,  Ω = [[0, −p, −q, −r], [p, 0, r, −q], [q, −r, 0, p], [r, q, −p, 0]]

**Rotational dynamics.** Euler's equation, J·ω̇ = −ω × (Jω) + M. With xz-plane symmetry the inertia matrix is

  J = [[Jx, 0, −Jxz], [0, Jy, 0], [−Jxz, 0, Jz]]

and B&M solve it in closed form with the Γ coefficients (Γ = Jx Jz − Jxz²):

  ṗ = Γ1 p q − Γ2 q r + Γ3 l + Γ4 n
  q̇ = Γ5 p r − Γ6 (p² − r²) + m / Jy
  ṙ = Γ7 p q − Γ1 q r + Γ4 l + Γ8 n

Assumptions: rigid body, constant mass (electric, so no fuel burn), xz-plane symmetry, flat non-rotating Earth.

## Why quaternions

Euler angles need tan θ and 1/cos θ in their kinematics, which blow up at θ = ±90°. A fixed-wing aircraft rarely goes there in normal flight, but a stall, a spin or a bug can, and the sim must not crash when it does. Quaternions have no singularity and cost one extra state. The one constraint, |e| = 1, drifts slowly under numerical integration, so the integrator renormalizes after every step. Euler angles are still computed for logging and for the autopilot, which thinks in φ, θ, ψ.

## Integrator

Classic fixed-step RK4, as in the B&M design projects. Error per unit time scales as dt⁴, so halving dt cuts it ~16×; a test checks exactly that. Fixed step (not adaptive) because the GNC loops and sensors will run at fixed rates later.

## How it is verified

No flight data exists, so every layer is checked against a case with a known answer (see `sim/tests/test_rigid_body.cpp`). For the rigid body:

1. **Analytic motion:** coasting, free fall at an arbitrary attitude, steady roll.
2. **Conservation:** with zero moment, angular momentum in NED and rotational KE are exact invariants. They hold to round-off (~1e-14) over 30 s at dt = 1 ms, including with Jxz ≠ 0.
3. **Known physics:** the intermediate-axis theorem. A spin about the axis with the middle moment of inertia (pitch, for the Aerosonde) is unstable and periodically flips; a spin about the largest axis is stable.
4. **Convergence order:** the RK4 error ratio when halving dt is ~16.

A test only counts if it can fail: flipping the sign of one Γ term, or one entry of Ω, makes several of these fail.
