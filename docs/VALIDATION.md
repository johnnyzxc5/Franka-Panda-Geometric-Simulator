# Numerical validation

Recorded on October 13, 2026 with MATLAB R2026a Update 4. Reports use fixed seed 13102026. Tests were run with a restored default MATLAB path plus this repository's setup; dependency analysis reported base MATLAB only.

## Independent checks

`validate_panda` samples 40 configurations inside the seven position limits. A COM-Jacobian calculation independent of the spatial recursion checks the mass matrix and gravity. Central differences of its mass and potential check Coriolis forces, the power identity and potential gradient. Other checks cover product-of-exponentials versus relative-chain kinematics, finite-difference Jacobians, body/space adjoints, positivity, inverse/forward dynamics with a flange wrench, friction dissipation, and cache invalidation after changing model geometry/screws. Additional inertia is tested as a separate optional extension.

| Check | Recorded maximum error |
|---|---:|
| Relative mass matrix difference | 5.68e-16 |
| Gravity against COM Jacobians | 2.21e-14 N m |
| Coriolis against Lagrange calculation | 5.77e-10 N m |
| Gravity against potential gradient | 2.61e-9 N m |
| Finite-difference twist Jacobian | 6.33e-11 |
| Forward/inverse acceleration round trip | 3.91e-13 rad/s² |
| Minimum sampled mass eigenvalue | 0.00356318 kg m² |

The independent computation checks mathematical consistency; it does not establish experimental accuracy. Parity with the bundled Modern Robotics routines is also checked, but is not treated as an independent physical validation. Exact values and pass thresholds are in `results/validation.json` and the test code.

`test_source_geometry` compares model relative frames, axes and flange pose with joint-origin facts extracted from manufacturer Xacro; maximum relative-transform discrepancy was 2.36e-17. `test_simulation_inputs` checks nonzero environmental work, 10% loss of effectiveness at joint 2, applied-effort bookkeeping, and saturation of all seven commands. These are separate from the nominal demonstration.

## Integration and energy

The nominal demonstration runs for 3.6 s, with a matching computed-torque controller at 4 ms. RK4 substeps of 1 ms and 0.5 ms are compared while keeping the control period unchanged. Maximum joint-position and velocity differences were 1.46e-7 rad and 3.89e-7 rad/s. The maximum mechanical energy-balance residual was 5.63e-7 J.

Joint position limits, velocity bounds and saturated command magnitudes passed their stage/sample checks. Recorded maximum joint error was about 0.087 degrees. This measures numerical execution of this model/controller pair; it is not a hardware tracking result. The energy residual did not materially decrease with refinement, so we claim a small checked residual and stable state integration, not a demonstrated fourth-order convergence rate.

## Timing

`benchmark_panda` warms both implementations, then times 100 changing q/qd pairs in five repeats, with positions inside the joint limits. Both compute the same complete mass matrix plus Coriolis, gravity and friction. The median reference time was 1255.0 microseconds and optimized time 180.4 microseconds, approximately 6.96x speedup on this machine. Rendering and integration are excluded. Warmed loops use changing states, so this is not a repeated-state cache-hit benchmark.

SHAPE kernels hoist invariant adjoints, reuse exact-state kinematics between mass/bias evaluation, compute zero-state mass recursion without unnecessary bias terms, and evaluate Lie brackets directly. No approximate lookup tables, generated MEX binaries or measured-state interpolation are used.

## Re-run

```matlab
setup_panda;
addpath('tests','tools');
validate_panda;
test_source_geometry;
test_simulation_inputs;
run_panda_demo;
benchmark_panda;
```

Readable receipts are versioned; the larger MAT simulation output is generated locally and ignored by Git. Machine-dependent timings may differ. Changed trajectories, forcing, model parameters or controller gains need their own integration check.
