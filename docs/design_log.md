# Design log

One entry per decision, newest at the bottom. "Rev" numbers match the project brief; "S" entries are simulator milestones. Numbers are simulation or estimate results, not measurements.

## Aircraft design (brief revisions)

| Rev | Date | Decision |
|---|---|---|
| 0 | 2026-09-28 | Mission and sizing: 1.8 m span, AR ~5.5, S ~0.59 m², ~2.4 kg, V_stall ~9 m/s at C_L,max 0.8. Rear folding pusher prop, elevons, 4S2P 21700 Li-ion (~130 Wh). EO + IR nose payload. Simulation-only, kept buildable. |
| 0.1 | 2026-09-28 | Loiter at 12 m/s (≈1.3 V_stall): the minimum-power C_L (~1.1) is above C_L,max, so the aircraft is stall-limited and should loiter as slowly as margin allows. Twin canted fins. Line-item mass budget. Static margin target 5–8% MAC. |
| 0.2 | 2026-09-28 | Dash 20 m/s; operate in ≥ 10 m/s steady wind. |
| 0.3 | 2026-09-28 | Airfoils: MH 45 outer wing, MH 91 center body (flow5 2D polars, Re 100k–900k). MH 45 over MH 60 for less nose-down C_m and higher C_L,max; MH 91's positive C_m offsets the outer wing. |
| 0.4 | 2026-09-29 | Planform variant C (flow5 VLM2): outer taper 0.70, 5.4° tip washout, MAC 0.366 m. Picked over taper 0.55 for better stall progression at the same drag. Elevons from 2y/b 0.55, outboard of predicted stall onset. |
| 0.5 | 2026-09-29 | Body-edge canted fins (35° cant, 55° LE sweep, NACA 0008): C_nβ +0.041 /rad. x_np 0.510 m, CG 0.489 m (SM 5.8%). All modes stable: short period ζ 0.68, Dutch roll ζ 0.10, spiral t½ 18 s. |
| 0.6 | 2026-09-29 | Elevon derivatives: C_mδe −0.334 /rad, C_lδa 0.198 /rad, C_nδa ≈ 0. Trim elevon −3.2° (9 m/s) to +3.6° (20 m/s). Mass v2: fins 120 g, Ixx/Iyy/Izz 0.180/0.098/0.273 kg·m². |
| 0.7 | 2026-09-29 | Propulsion sized from the drag polar with a BEMT prop model calibrated to APC data. |
| 0.8 | 2026-09-29 | Motor: SunnySky X2820 800KV + 11x8 folding prop + 40 A ESC. Loiter ~132 min, reference mission ~100 min (74 min on station), climb 5.4 m/s. |

## Simulator

### S-0.1: rigid-body core (2026-09-29)

**What:** 13-state rigid-body equations of motion (B&M ch. 3) with a quaternion attitude (B&M App. B), fixed-step RK4, and verification tests. Forces and moments are inputs, so aero, thrust and gravity plug in later without touching the core. Derivation and design choices: [`sim/01_rigid_body.md`](sim/01_rigid_body.md).

**Verified (all pass, `ctest`):**
- Euler ↔ quaternion round trip and quaternion DCM vs. the 3-2-1 Euler DCM, to 1e-12.
- Coasting with no force: straight line at constant attitude.
- Free fall at an arbitrary attitude: p_d = g t²/2.
- Steady roll about a principal axis: φ = p t.
- Torque-free tumble (Aerosonde and BWB inertias, J_xz ≠ 0): angular momentum in NED and rotational KE conserved to 1e-8 over 20 s.
- Intermediate-axis theorem: spin about pitch flips, spin about yaw stays put.
- RK4 convergence: halving dt cuts the error ~16×.

**Decisions:**
- Quaternions instead of Euler angles for the state: no singularity at θ = ±90°, and it matches the B&M companion code so the Aerosonde comparison lines up state by state.
- No external math library. The state is 13 numbers; small hand-written Vec3/Mat3/Quat types are easy to read and test.
- BWB J_xz sign: the inertia script uses geometry axes (x aft), so its −0.005 kg·m² becomes +0.005 in B&M body axes (x forward).

**Open:**
- Mass/inertia v3: fold in the Rev 0.8 motor (+28 g aft) and the battery move to x ≈ 0.418 m.
- Confirm the Aerosonde mass (11.0 kg in the companion code vs 13.5 kg in the 1st-edition table).

**Next (S-0.2):** Aerosonde forces and moments (B&M ch. 4): gravity, linear aero model with stall blending, propeller, then trim (ch. 5) and compare with the reference code.
