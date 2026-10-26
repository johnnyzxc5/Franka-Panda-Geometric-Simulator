# Final Mathematical and Scientific Review (HITO 6 Quality Audit)

**Milestone:** HITO 6 — Revisión matemática y científica final  
**Date:** October 26, 2026  
**Author / Model:** Argon (Gemini 4 Argon)  
**Scope:** End-to-end audit of mathematical fidelity, joint limit topology, dynamic equivalence, Lie-group integration, benchmark interpretation, and publication readiness of Version A (`+panda/`) and Version B (`+panda_s1/`).

---

## 1. Executive Verdict

Both formulations of the 7-DoF Franka Emika Panda simulator — **Version A (`+panda/`, Euclidean $q \in \mathbb{R}^7$)** and **Version B (`+panda_s1/`, Global Lie-Group / Manifold $X \in (S^1)^7$)** — have passed all mathematical, physical, numerical, and regression audits with zero open defects. Both implementations are verified and ready for public release and side-by-side presentation in **Milestone 7**.

---

## 2. Critical Audit across the Eight Mandatory Review Axes

### 2.1 Fidelity of the Geometric Representation on $(S^1)^7$
- **Manifold & Tangent Bundle Structure:**  
  Each revolute joint is represented by $x_i = (c_i, s_i)^T \in S^1 \subset \mathbb{R}^2$ ($X = [x_1, \dots, x_7] \in \mathbb{R}^{2\times 7}$), with left-trivialized tangent bundle $T(S^1)^7 \cong (S^1)^7 \times \mathbb{R}^7$ governed by $\dot{x}_i = \omega_i J x_i$, matching Lee, Leok & McClamroch (2018, Chapter 4, Eq. 4.1) and Pinocchio's `JointModelRevoluteUnbounded`.
- **Purely Polynomial Kinematics:**  
  In `+panda_s1/screw_exp.m`, `+panda_s1/fk.m`, `+panda_s1/frames.m`, `+panda_s1/jacobian.m`, and `third_party/shape_kernels/rnea_shared_kinematics_S1.m`, all nominal spatial transformations, link frames, twist Jacobians, and inter-link Adjoint matrices are evaluated as **purely polynomial functions of $(c_i, s_i)$**, with zero calls to `to_q`, `atan2`, `cos`, or `sin`.

### 2.2 Validity of Bounded Joint Constraints & Topological Analysis
- **Algebraic Characterization:**  
  Because all 7 Panda joints satisfy $0 < h_i = (q_{\max,i} - q_{\min,i})/2 < \pi$, the admissible arc $\mathcal{A}_i \subset S^1$ is exact and unambiguous via $x_i^T x_i = 1$ and $u_i^T x_i \ge \cos h_i$ (`+panda_s1/check_limits.m`).
- **Branch-Cut-Safe Centered Lift (`to_q.m`):**  
  Rotating $x_i$ into the frame of the arc midpoint $u_i = (\cos m_i, \sin m_i)^T$ places the branch cut of $\operatorname{atan2}$ at $m_i \pm \pi$, which is the exact midpoint of the forbidden mechanical sector $S^1 \setminus \mathcal{A}_i$. This enables completely smooth traversal of the principal branch cut $+\pi$ on Joint 6 ($[-0.0175, +3.7525]\text{ rad}$), verified to $4.44 \times 10^{-15}\text{ rad}$ trajectory parity and $1.62 \times 10^{-11}\text{ J}$ energy residual.
- **Topological Hazard of Geodesic Errors When $2h_i > \pi$ (`geodesic_diff` vs. `box_diff`):**  
  Because 6 out of 7 Panda joints have admissible span $2h_i > \pi$, the forbidden complement arc $S^1 \setminus \mathcal{A}_i$ is shorter than $\pi$. Consequently, when $|\Delta q_i| > \pi$, the shortest geodesic on $S^1$ (`geodesic_diff`) and Lee's chordal error $\sin(q_{2,i} - q_{1,i})$ point **across the physical hard stop** (experimentally shown to cause a $0.2433\text{ rad}$ stop violation on Joint 1 in `test_s1_simulation_consistency.m`). Using the homotopy-consistent admissible-arc displacement `box_diff` guarantees that interpolation (`+panda_s1/reference.m`) and PD control remain strictly inside $\mathcal{A}_i$.

### 2.3 Dynamic Equivalence & Preservation of Physical Parameters
- **Unaltered Physical Model:**  
  `+panda_s1/model.m` inherits the exact Gaz et al. (2019) inertial parameters, modified DH parameters, gravity vector, sigmoid friction model, apparent rotor inertia $J_{\text{app}}$, and joint bounds from `+panda/model.m` (`sources/parameters.json`).
- **Exact Equivalence of Lee's Reduced Equations and Spatial RNEA:**  
  The directional derivative operator $\mathcal{D}_i = (J x_i)^T \frac{\partial}{\partial x_i}$ on $(S^1)^7$ is the pushforward of $\frac{\partial}{\partial q_i}$. Across 100 random states (`benchmark_s1_comparison.m`) and 40 validation states (`validate_panda_s1.m`):
  - Mass matrix $M(X)$ vs. independent Lagrange oracle on $(S^1)^7$: $5.64 \times 10^{-16}$.
  - Mass matrix $M(X)$ vs. Version A $M(q)$: $8.88 \times 10^{-16}\text{ kg m}^2$.
  - Inverse dynamics $\tau(X,\omega,\dot{\omega})$ vs. Version A: $2.13 \times 10^{-14}\text{ N m}$.
  - Tangential finite-difference Christoffel Coriolis & potential gradient on $(S^1)^7$: verified to $4.62 \times 10^{-10}\text{ N m}$ and $2.42 \times 10^{-9}\text{ N m}$.

### 2.4 Correctness of the Lie-Group Integrator (RK4-MK)
- **Abelian Munthe–Kaas RK4:**  
  Because $\mathfrak{so}(2)^7$ is abelian, $\mathrm{dexp}_{-\Delta\theta}^{-1}(\omega) = \omega$. Updating stages via $X^{(r)} = \operatorname{Exp}_{S^1}(\Delta\theta^{(r)}, X_k)$ with 1-ulp radial normalization preserves $|c_i^2 + s_i^2 - 1| \le 2.22 \times 10^{-16}$ over 5,000+ steps without phase distortion (whereas unprojected ambient $\mathbb{R}^{14}$ RK4 drifts by $7.87 \times 10^{-9}$).
- **Isomorphism with Lifted RK4:**  
  Across the full 3.6 s trajectory (3,600 substeps), the maximum trajectory discrepancy between Lie-group RK4-MK on $(S^1)^7$ (`Version B`) and Euclidean RK4 on $\mathbb{R}^7$ with smooth trig kernels (`Variant B-alg`) is **$4.44 \times 10^{-15}\text{ rad}$ ($2\,\text{ulp}$)**.

### 2.5 Audit and Correction of Hito 5 Benchmark Interpretation
During the Hito 6 audit of Hito 5, two important refinements were made and verified:
1. **Addition of the Control Benchmark (`Variant B-alg`) to Separate Algorithmic vs. Representational Gains:**  
   In Hito 5, `benchmark_s1_comparison.m` initially compared Stock MR, Version A, and Version B without the intermediate control **Variant B-alg** ($q \in \mathbb{R}^7$ + specialized algebraic $S^1$ kernels). We added `Variant B-alg` to `tools/benchmark_s1_comparison.m`, reran the full 1,000-call campaign, and generated the 4-panel figure `results/benchmark_s1_comparison.png`.
2. **Elimination of Redundant Stage `atan2` Calls in `+panda_s1/simulate.m`:**  
   By exploiting the strict monotonicity of $\cos|\delta_i| = u_i^T x_i$ on $[0, \pi]$, stage limit monitoring inside the 3,600-substep integration loop now tracks the minimum algebraic dot product $u_i^T x_i$ on $S^1$ and evaluates `atan2` only once after the loop on the 7 worst-case stage configurations, yielding identical bit-for-bit stage margin metrics with cleaner execution.

---

## 3. Honest Scientific Synthesis: Real Advantages vs. Conceptual Trade-Offs (No Forced Victory)

| Criterion | Winner / Assessment | Rigorous Scientific Explanation |
|:---|:---:|:---|
| **1. Speed of Individual Kinematic & RNEA Operators** | **Version B ($(S^1)^7$)**<br>*(Mostly Algorithmic, +5–12% Representational)* | Version B is **$3.43\times$ faster** in `fk`, **$6.58\times$ faster** in `frames`, **$2.21\times$ faster** in `inverse_dynamics`, and **$1.51\times$ faster** in `dynamics` than Version A. Comparing against `Variant B-alg` shows that **$\sim 90\%$ of this speedup** comes from replacing generic `MatrixExp6`/dense `Adjoint` with Theorem 3.1 and the hoisted adjoint homomorphism $B^{(i)} = \operatorname{Ad}_{M_i^{-1}}$ (which also benefits $q \in \mathbb{R}^7$), while **$\sim 5\%\text{–}12\%$ ($0.6\text{–}3.6\,\mu\text{s}$ per call)** is a pure representational gain from already having $(c_i, s_i)$ in memory without calling `cos(q)` / `sin(q)`. |
| **2. Closed-Loop Simulation Speed (3.6 s, 3,600 RK4 steps)** | **Variant B-alg ($1.82\times$ RTF)** closely followed by **Version B ($1.75\times$ RTF)** vs. Version A ($1.24\times$ RTF) | Both Version B ($2.06\text{ s}$) and Variant B-alg ($1.97\text{ s}$) are **$>1.41\times$ faster** than Version A ($2.91\text{ s}$). Integrating on $(S^1)^7$ via RK4-MK adds a modest **$4.4\%$ ($24.4\,\mu\text{s}$/step) overhead** relative to Variant B-alg because RK4-MK evaluates `exp_s1` ($\cos\Delta\theta, \sin\Delta\theta$ + $SO(2)$ rotation + radial normalization) at the 4 RK4 stages, whereas Euclidean RK4 adds angles linearly ($q + \Delta\theta$) and evaluates $\cos q, \sin q$ at the stages. |
| **3. Energy Balance & Step-Size Convergence ($O(h^4)$)** | **Tie: Version B & Variant B-alg** (**$5,621\times$ to $2.3\times 10^6\times$ better** than Version A) | Version A's `MatrixExp6` suffers from a $10^{-6}\text{ rad}$ `NearZero` branch cutoff that snaps $R_i \to I_3$ near $q_i = 0$, trapping Version A's energy residual at $5.62 \times 10^{-7}\text{ J}$ regardless of step size $h$. Both Version B (algebraic on $S^1$) and Variant B-alg (direct IEEE-754 `cos`/`sin` without `NearZero`) are $C^\infty$ at $q_i = 0$ and achieve **$1.00 \times 10^{-10}\text{ J}$** at $h=1\text{ ms}$ and **$2.44 \times 10^{-13}\text{ J}$** at $h=0.25\text{ ms}$. |
| **4. Polynomial Structure & Analytical Differentiation** | **Version B ($(S^1)^7$)** | On $(S^1)^7$, $T(X)$, $J(X)$, $M(X)$, $c(X,\omega)$, and $g(X)$ are **purely polynomial in the configuration state $X$** (zero transcendental functions of configuration), which is a genuine structural advantage for polynomial optimization (SOS), symbolic algebra, and analytical derivatives. |
| **5. Global Topology vs. Bounded Joint Stops** | **Nuanced Trade-Off** | For *unbounded* revolute joints, $S^1$ eliminates $\pm\pi$ wrapping discontinuities globally. For a *bounded* arm where 6/7 joints have span $2h_i > \pi$, naive $S^1$ geodesics (`geodesic_diff`) can cross the forbidden stop; safe large-displacement control requires the centered admissible-arc lift `box_diff`, which places the branch cut inside the stop ($m_i \pm \pi$) and smoothly handles Joint 6 crossing $+\pi$. |

---

## 4. Verification of Readiness for Milestone 7 (Public Release)

1. **Version A (`+panda/`):** 100% preserved (`git diff main -- +panda/` is empty), all original validation suites (`validate_panda`, `test_source_geometry`, `test_simulation_inputs`, `verify_sources.py`) pass with zero regressions.
2. **Version B (`+panda_s1/`):** Complete, self-contained, and verified across all geometry, limit, dynamics, simulation, and benchmark suites (`test_s1_geometry_limits`, `validate_panda_s1`, `test_s1_simulation_consistency`, `benchmark_s1_comparison`).
3. **Documentation & Artifacts:** Mathematical formulation, implementation architecture, integration/constraint analysis, 4-way benchmark data/plots (`results/benchmark_s1_comparison.png`), and this final scientific audit are complete and committed.
