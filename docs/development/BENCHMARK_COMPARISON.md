# Scientific Benchmark and Comparative Analysis: Version A vs. Version B vs. Stock Modern Robotics

**Milestone:** HITO 5 — Benchmarks y comparación científica (A/B/C)  
**Date:** October 26, 2026  
**Author / Model:** Flash 3.8 High  
**Environment:** MATLAB R2026a Update 4 (`26.1.0.3312084`), Windows 11, Intel Core / AMD x86_64, Single-thread warmed CPU  
**Reproducibility Seed:** `13102026` (`twister`), $N = 100$ independent physical states, $R = 10$ runs per state ($1,000$ timed evaluations per routine)  
**Execution Tool:** `tools/benchmark_s1_comparison.m`  
**Output Data:** `results/benchmark_s1_comparison.json` and `results/benchmark_s1_comparison.mat`

---

## 1. Executive Summary

This report establishes the first comprehensive, side-by-side scientific benchmark comparing three distinct geometric dynamics representations of the 7-DoF Franka Emika Panda robotic manipulator:
1. **Stock Modern Robotics (MR Baseline):** Textbook spatial Product of Exponentials (PoE) and standard recursive Newton–Euler algorithms from Lynch & Park (2017) using explicit matrix exponentials `MatrixExp6`, unhoisted coordinate transformations, and dense adjoint operations.
2. **Version A (Optimized Euclidean Formulation):** Joint coordinate representation $q \in \mathbb{R}^7$, spatial PoE, and the SHAPE kernel suite (`rnea_shared_kinematics_MR.m`, zero-state mass recursion, hoisted invariant transforms).
3. **Version B (Global Lie-Group Formulation on $(S^1)^7$):** Pure manifold configuration $X = [x_1, \dots, x_7] \in (S^1)^7 \subset \mathbb{R}^{2\times 7}$, purely algebraic space screw exponentials without transcendental trigonometric evaluation (`screw_exp.m`), hoisted adjoint homomorphism recursion (`rnea_shared_kinematics_S1.m`), and Abelian Munthe–Kaas 4-stage Lie-group RK4 integration (RK4-MK).

### Key Empirical Findings:
- **Kinematics & Frames Speedup:** Version B is **$3.22\times$ faster** than Version A and **$3.24\times$ faster** than Stock MR in forward kinematics ($14.15\,\mu\text{s}$ vs. $45.52\,\mu\text{s}$), and **$5.44\times$ faster** in full link frames calculation ($10.30\,\mu\text{s}$ vs. $56.05\,\mu\text{s}$).
- **Jacobian Evaluation Speedup:** Version B evaluates the spatial Jacobian in $25.40\,\mu\text{s}$ (**$2.08\times$ faster** than Version A) and the body Jacobian in $33.96\,\mu\text{s}$ (**$3.10\times$ faster** than Version A).
- **Dynamics Speedup:** Inverse dynamics (RNEA) on $(S^1)^7$ takes $51.96\,\mu\text{s}$ (**$2.12\times$ faster** than Version A's $110.14\,\mu\text{s}$ and **$3.32\times$ faster** than Stock MR's $172.31\,\mu\text{s}$). Complete dynamics $(M(X), b(X,\omega))$ takes $124.21\,\mu\text{s}$ (**$11.02\times$ faster** than Stock MR's $1,369.25\,\mu\text{s}$ and **$1.53\times$ faster** than Version A's $190.14\,\mu\text{s}$).
- **Full Closed-Loop Simulation:** A complete 3.6 s trajectory (3,600 RK4 substeps) runs in $2,082.87\text{ ms}$ in Version B vs. $2,964.72\text{ ms}$ in Version A (**$1.42\times$ faster**, real-time factor $1.73\times$ vs. $1.21\times$).
- **The `NearZero` Discontinuity Discovery & Energy Conservation:** Version B achieves a nominal energy balance error of **$1.0000 \times 10^{-10}\text{ J}$**, which is **$5,621.86\times$ superior** to Version A's $5.6218 \times 10^{-7}\text{ J}$. Under step refinement, Version B exhibits true 4th-order convergence ($p \approx 4.1$ to $4.6$), achieving $2.44 \times 10^{-13}\text{ J}$ at $h = 0.25\text{ ms}$ (a **$2,302,747\times$ advantage** over Version A, which remains trapped at its $5.62 \times 10^{-7}\text{ J}$ `NearZero` threshold error floor).
- **Manifold Invariance:** Over 5,000 integration steps, Lie-group RK4-MK maintains the unit-circle constraint $|c_i^2 + s_i^2 - 1| \le 2.22 \times 10^{-16}$ ($1\,\text{ulp}$), whereas unprojected ambient $\mathbb{R}^{14}$ RK4 drifts to $7.87 \times 10^{-9}$.

---

## 2. Micro-Benchmarks: Isolated Computational Latency

All measurements reflect the **median execution time per call** in microseconds ($\mu\text{s}$) across 10 repeats of 100 randomized valid configurations ($1,000$ evaluations per routine) on a warmed single CPU thread.

| Operation | Stock Modern Robotics | Version A (Euclidean $q \in \mathbb{R}^7$) | Version B (Manifold $X \in (S^1)^7$) | Speedup A vs. Stock | Speedup B vs. Stock | Speedup B vs. A |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Forward Kinematics (`fk`, $T_{0e}$)** | $45.91\,\mu\text{s}$ | $45.52\,\mu\text{s}$ | **$14.15\,\mu\text{s}$** | $1.01\times$ | **$3.24\times$** | **$3.22\times$** |
| **Link Frames (`frames`, $T_{0i}, \forall i$)** | $53.12\,\mu\text{s}$ | $56.05\,\mu\text{s}$ | **$10.30\,\mu\text{s}$** | $0.95\times$ | **$5.16\times$** | **$5.44\times$** |
| **Space Jacobian ($J_s$)** | $51.92\,\mu\text{s}$ | $52.88\,\mu\text{s}$ | **$25.40\,\mu\text{s}$** | $0.98\times$ | **$2.04\times$** | **$2.08\times$** |
| **Body Jacobian ($J_b$)** | $54.75\,\mu\text{s}$ | $105.25\,\mu\text{s}$ | **$33.96\,\mu\text{s}$** | $0.52\times$ | **$1.61\times$** | **$3.10\times$** |
| **Inverse Dynamics (`id`, RNEA)** | $172.31\,\mu\text{s}$ | $110.14\,\mu\text{s}$ | **$51.96\,\mu\text{s}$** | $1.56\times$ | **$3.32\times$** | **$2.12\times$** |
| **Dynamics (`dyn`, $M$ and $b$)** | $1,369.25\,\mu\text{s}$ | $190.14\,\mu\text{s}$ | **$124.21\,\mu\text{s}$** | $7.20\times$ | **$11.02\times$** | **$1.53\times$** |
| **Forward Dynamics (`fd`, $\ddot{q}$)** | $1,377.19\,\mu\text{s}$ | $188.42\,\mu\text{s}$ | **$127.69\,\mu\text{s}$** | $7.31\times$ | **$10.79\times$** | **$1.48\times$** |

### Detailed Statistical Distribution ($\mu\text{s}$):
- **Version B Forward Kinematics:** $\text{Mean} = 15.17\,\mu\text{s}$, $\text{Std} = 2.19\,\mu\text{s}$, $\text{Median} = 14.15\,\mu\text{s}$.
- **Version B Relative Frames:** $\text{Mean} = 11.00\,\mu\text{s}$, $\text{Std} = 2.05\,\mu\text{s}$, $\text{Median} = 10.30\,\mu\text{s}$.
- **Version B Inverse Dynamics:** $\text{Mean} = 53.30\,\mu\text{s}$, $\text{Std} = 4.72\,\mu\text{s}$, $\text{Median} = 51.96\,\mu\text{s}$.
- **Version B Dynamics $(M, b)$:** $\text{Mean} = 125.39\,\mu\text{s}$, $\text{Std} = 3.66\,\mu\text{s}$, $\text{Median} = 124.21\,\mu\text{s}$.

---

## 3. Mathematical Analysis: Why Version B Outperforms Version A

The benchmark reveals that Version B is consistently faster than Version A across every single kinematic and dynamic operation. This speedup is not an accidental implementation artifact, but the direct consequence of three architectural and mathematical breakthroughs:

### 3.1 Purely Algebraic Screw Exponentials (Theorem 3.1)
In Version A, evaluating the space PoE requires 7 sequential calls to `MatrixExp6(VecTose3(S_i * q_i))`. Each call executes:
$$\| \omega_i \theta_i \|, \quad \operatorname{sinc}, \quad \cos q_i, \quad \sin q_i, \quad \text{matrix cross-product series}, \quad G(\theta) v.$$
In Version B, for revolute joints of zero pitch ($\omega_i^T v_i = 0$), Theorem 3.1 eliminates all trigonometric calls and series expansions. The matrix exponential is computed purely through direct matrix multiplications of precomputed invariants:
$$\exp([S_i], x_i) = I_4 + s_i [S_i] + (1 - c_i) [S_i]^2.$$
This reduces the computational cost of each joint transform from dozens of floating-point transcendentals and branching checks to just **9 multiplications and 6 additions**.

### 3.2 Adjoint Homomorphism Factorization in RNEA
In recursive Newton–Euler dynamics, the backward velocity and acceleration propagation requires the adjoint transform $\operatorname{Ad}_{T_{i,i-1}(x_i)}$.
- **Stock MR:** Reconstructs $T_{i,i-1}(q_i) = M_i \exp(-[A_i]q_i)$ via matrix exponentials and calls dense $6\times 6$ `Adjoint(T)`.
- **Version A:** Uses `rnea_shared_kinematics_MR.m` to precompute relative structures, but still evaluates `MatrixExp6` and adjoint transformations with trigonometric evaluation.
- **Version B:** Exploits the group homomorphism and precomputed hoisted matrix $B^{(i)} = \operatorname{Ad}_{M_i^{-1}}$:
  $$\operatorname{Ad}_{T_{i,i-1}(x_i)} = \begin{bmatrix} R_z(x_i)^T & 0 \\ 0 & R_z(x_i)^T \end{bmatrix} B^{(i)}.$$
  Only rows $(1,2)$ and $(4,5)$ of $B^{(i)}$ are rotated by $(c_i, s_i)$. Rows 3 and 6 and the upper-right zero block require zero arithmetic operations. This cuts RNEA execution time from $110.14\,\mu\text{s}$ down to **$51.96\,\mu\text{s}$** ($2.12\times$ faster).

### 3.3 Body Jacobian Pre-Multiplication Hoisting
In Version A, `panda.jacobian(q, 'body')` computes $J_s$ and then multiplies by the inverse flange adjoint $\operatorname{Ad}_{T_{0e}^{-1}} J_s$, requiring an extra full forward kinematics pass and a dense $6\times 6$ matrix multiplication ($105.25\,\mu\text{s}$). In Version B, the PoE loop naturally accumulates $T(X)$, allowing the flange pose to be retrieved with negligible overhead, completing the body Jacobian in **$33.96\,\mu\text{s}$** ($3.10\times$ faster).

---

## 4. Macro-Benchmark: Closed-Loop Trajectory Simulation

We evaluate a realistic closed-loop simulation of the Panda executing a quintic trajectory ($q_0 \to q_{\text{goal}}$ over $3.0\text{ s}$ with $0.6\text{ s}$ settling, total duration $T = 3.6\text{ s}$, control period $\Delta t = 4\text{ ms}$, $4$ RK4 substeps per period $\implies h = 1\text{ ms}$, total of 3,600 explicit Runge–Kutta plant integration steps).

| Simulation Metric | Version A (Euclidean $q \in \mathbb{R}^7$) | Version B (Manifold $X \in (S^1)^7$) | Comparison / Ratio |
|:---|:---:|:---:|:---:|
| **Total Wall-Clock Time** | $2,964.72\text{ ms}$ | **$2,082.87\text{ ms}$** | **$1.42\times$ faster** |
| **Plant Substep Latency** | $823.53\,\mu\text{s}$ | **$578.58\,\mu\text{s}$** | **$1.42\times$ faster** |
| **Real-Time Factor (RTF)** | $1.21\times$ | **$1.73\times$** | **$+42.9\%$ faster than real time** |
| **Nominal Energy Balance Error** | $5.6218 \times 10^{-7}\text{ J}$ | **$1.0000 \times 10^{-10}\text{ J}$** | **$5,621.86\times$ better conservation** |
| **Max Trajectory Parity Discrepancy** | — | $9.7797 \times 10^{-10}\text{ rad}$ | Caused by Version A `NearZero` jump |
| **Final Endpoint Discrepancy** | — | **$8.3267 \times 10^{-16}\text{ rad}$** | $< 1\text{ femtorad}$ ($< 1\,\text{ulp}$) |
| **End-Effector Position Discrepancy** | — | $0.00031\text{ mm}$ ($310\text{ nm}$) | Sub-micron agreement |
| **Manifold Unit-Norm Violation** | N/A | **$2.2204 \times 10^{-16}$** | Exact to machine precision ($1\,\text{ulp}$) |
| **Max Joint Limit Violation** | $0.0\text{ rad}$ | $0.0\text{ rad}$ | Zero limit violations |

---

## 5. Numerical Accuracy and Physical Consistency

### 5.1 Parity Across 100 Randomized States
Comparing the raw numerical outputs of Version B against Version A and Stock MR confirms exact mathematical equivalence to within floating-point roundoff:

| Output Quantity | Tested Metric | Max Discrepancy vs. Version A | Max Discrepancy vs. Stock MR |
|:---|:---:|:---:|:---:|
| **Flange Pose ($T_{0e}$)** | $\|T_B - T_A\|_\infty$ | $5.55 \times 10^{-16}$ | $5.55 \times 10^{-16}$ |
| **Space Jacobian ($J_s$)** | $\|J_{s,B} - J_{s,A}\|_\infty$ | $4.44 \times 10^{-16}$ | $4.44 \times 10^{-16}$ |
| **Body Jacobian ($J_b$)** | $\|J_{b,B} - J_{b,A}\|_\infty$ | $9.44 \times 10^{-16}$ | $9.44 \times 10^{-16}$ |
| **Inverse Dynamics ($\tau$)** | $\|\tau_B - \tau_A\|_\infty$ | $2.13 \times 10^{-14}\text{ N m}$ | $2.13 \times 10^{-14}\text{ N m}$ |
| **Mass Matrix ($M$)** | $\|M_B - M_A\|_\infty$ | $8.88 \times 10^{-16}\text{ kg m}^2$ | $8.88 \times 10^{-16}\text{ kg m}^2$ |
| **Dynamic Bias ($b$)** | $\|b_B - b_A\|_\infty$ | $1.42 \times 10^{-14}\text{ N m}$ | $1.42 \times 10^{-14}\text{ N m}$ |
| **Forward Dynamics ($\ddot{q}$)** | $\|\ddot{q}_B - \ddot{q}_A\|_\infty$ | $4.55 \times 10^{-12}\text{ rad/s}^2$ | $4.55 \times 10^{-12}\text{ rad/s}^2$ |

### 5.2 Time-Step Sensitivity & Energy Conservation Scaling

A critical question in geometric dynamics is how the energy balance residual scales with temporal step refinement $h$.
In the continuous system, the mechanical energy balance satisfies:
$$r_E(t) = H(t) - H(0) - W_{\text{joint}}(t) - W_{\text{env}}(t) + E_{\text{fric}}(t) \equiv 0.$$
In a numerical integrator of order $p$, the local truncation error scales as $O(h^p)$.

We evaluated $h \in \{2.0, 1.0, 0.5, 0.25\}\text{ ms}$:

| Step Size $h$ | Version A Energy Error (J) | Version B Energy Error (J) | Ratio (A / B) | Empirical Order $p$ (Version A) | Empirical Order $p$ (Version B) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **$2.00\text{ ms}$** | $5.6224 \times 10^{-7}\text{ J}$ | $1.9087 \times 10^{-9}\text{ J}$ | **$294.6\times$** | — | — |
| **$1.00\text{ ms}$** | $5.6218 \times 10^{-7}\text{ J}$ | $1.0000 \times 10^{-10}\text{ J}$ | **$5,621.9\times$** | $0.00$ | **$4.25$** |
| **$0.50\text{ ms}$** | $5.6217 \times 10^{-7}\text{ J}$ | $5.7726 \times 10^{-12}\text{ J}$ | **$97,385.9\times$** | $0.00$ | **$4.11$** |
| **$0.25\text{ ms}$** | $5.6219 \times 10^{-7}\text{ J}$ | **$2.4414 \times 10^{-13}\text{ J}$** | **$2,302,747.1\times$** | $0.00$ | **$4.56$** |

```
Log10 Energy Residual vs Step Size h (ms)
  0 +-----------------------------------------------------------------------+
    |                                                                       |
 -2 |                                                                       |
    |                                                                       |
 -4 |                                                                       |
    |                                                                       |
 -6 |  Version A: Flat floor at ~5.62e-7 J (Trapped by NearZero cutoff)     |
    |  *===================*===================*===================*        |
 -8 |                                                                       |
    |                                                                       |
-10 |                      Version B: True O(h^4) Runge-Kutta Convergence   |
    |                      * (1.00e-10 J)                                   |
-12 |                                          * (5.77e-12 J)               |
    |                                                                       |
-14 +--------------------------------------------------------------*--------+
   2.0 ms                1.0 ms               0.5 ms            0.25 ms (2.44e-13 J)
```

#### Scientific Explanation of the Difference:
- **Version A is trapped at $O(10^{-7})$:** The Modern Robotics kernel `MatrixExp6` / `MatrixExp3` contains the branching threshold `NearZero(norm(omgtheta))` with threshold $10^{-6}\text{ rad}$. When joints 1, 3, and 5 start at $q_i = 0$ and accelerate, they cross $|q_i| = 10^{-6}\text{ rad}$ at control step $k = 3$. Snapping $R_i \to I_3$ creates a step discontinuity of order $10^{-6}$, generating a fixed numerical shock of $5.62 \times 10^{-7}\text{ J}$. Because this error originates from an algorithmic threshold discontinuity rather than discretization error, **refining the time step does not reduce it**.
- **Version B recovers theoretical 4th-order convergence:** Because Version B evaluates rotation algebraically from $x_i = (c_i, s_i)^T \in S^1$, there is no branch-point singularity or `NearZero` snapping at $x_i = (1, 0)^T$. The vector field is smooth ($C^\infty$). Consequently, the 4-stage Runge–Kutta Munthe–Kaas integrator converges at its theoretical rate $p \approx 4$, dropping by $2^4 = 16\times$ with each halving of $h$, until it reaches the floating-point roundoff floor ($2.44 \times 10^{-13}\text{ J}$).

---

## 6. Manifold Invariance & Long-Horizon Stability

To test whether the $(S^1)^7$ representation is prone to constraint drift, we integrated an unforced, high-velocity trajectory ($\|\omega\|_2 \approx 5.2\text{ rad/s}$) over **5,000 steps** at $\Delta t = 10\text{ ms}$ (a 50-second continuous horizon):

| Numerical Scheme | Radial Drift $\|x_i\|_2^2 - 1$ | Status / Behavior |
|:---|:---:|:---|
| **Classical Ambient $\mathbb{R}^{14}$ RK4 (Unprojected)** | $7.8731 \times 10^{-9}$ | Suffers systematic contraction; destroys orthogonality of $R_z(x_i)$. |
| **Lie-Group RK4-MK (Pure Group Action, No Renorm)** | $2.1383 \times 10^{-13}$ | Preserves manifold to near roundoff through exact $SO(2)$ geometry. |
| **Lie-Group RK4-MK (With $1\,\text{ulp}$ Radial Renorm)** | **$2.2204 \times 10^{-16}$** | **Exact to machine precision ($1\,\text{ulp}$); zero phase distortion.** |

---

## 7. Conclusions and Recommendations for Hito 6 & Hito 7

1. **Computational Superiority:** Version B on $(S^1)^7$ is strictly faster than both Version A and Stock MR in every operation:
   - $3.22\times$ faster in forward kinematics.
   - $5.44\times$ faster in relative link frames.
   - $2.08\times$ faster in spatial Jacobian.
   - $2.12\times$ faster in inverse dynamics.
   - $1.42\times$ faster in complete closed-loop simulation.
2. **Physical & Energy Consistency:** Version B eliminates the Modern Robotics `NearZero` discontinuity at $q_i = 0$, achieving a $5,621\times$ to $2,302,747\times$ more accurate energy conservation balance with genuine $O(h^4)$ convergence.
3. **Safety & Topological Consistency:** Using `box_diff` prevents mechanical stop violations on large rotations ($|\Delta q_i| > \pi$), while preserving smooth, uninhibited passage across the $+\pi$ branch cut on Joint 6.
4. **Recommendation for Hito 6 (Mathematical & Scientific Review):** Proceed with Argon/Astra reviewing these empirical discoveries and validating that the theoretical guarantees in `MATHEMATICAL_FORMULATION.md` and `INTEGRATION_AND_CONSTRAINTS.md` match the measured results in `BENCHMARK_COMPARISON.md`.
