# Scientific Benchmark and Comparative Analysis: Stock MR vs. Version A vs. Variant B-alg vs. Version B

**Milestone:** HITO 5 & HITO 6 — Benchmarks y comparación científica (A / B-alg / B / Stock MR)  
**Date:** October 26, 2026  
**Authors / Models:** Flash 3.8 High (Initial Campaign) & Argon / Gemini 4 Argon (Four-Way Control Decomposition & Audit)  
**Environment:** MATLAB R2026a Update 4 (`26.1.0.3312084`), Windows 11, Single-thread warmed CPU  
**Reproducibility Seed:** `13102026` (`twister`), $N = 100$ independent physical states, $R = 10$ runs per state ($1,000$ timed evaluations per routine)  
**Execution Tool:** `tools/benchmark_s1_comparison.m`  
**Output Artifacts:** `results/benchmark_s1_comparison.json`, `results/benchmark_s1_comparison.mat`, and `results/benchmark_s1_comparison.png`

---

## 1. Experimental Design: Four-Way Scientific Decomposition

To rigorously answer whether performance and numerical improvements come from **algorithmic specialization** (replacing generic 6D matrix exponentials with specialized revolute rotations and hoisted adjoints) or from the **global manifold representation** ($X \in (S^1)^7$ vs. $q \in \mathbb{R}^7$), `tools/benchmark_s1_comparison.m` evaluates four distinct implementations on identical physical states:

1. **Stock Modern Robotics (MR Textbook Baseline):**  
   Standard `MatrixExp6`, `JacobianSpace`, `JacobianBody`, `InverseDynamics`, and `MassMatrix` from Lynch & Park (2017).
2. **Version A (`+panda` — Original Optimized Euclidean Formulation):**  
   Joint coordinates $q \in \mathbb{R}^7$, `MatrixExp6` PoE, and the SHAPE RNEA kernel (`rnea_shared_kinematics_MR.m`, zero-state mass recursion, hoisted constant transforms).
3. **Variant B-alg (`q` + Specialized Algebraic Kernels — Control Benchmark):**  
   Retains Euclidean coordinates $q \in \mathbb{R}^7$ as input and integration state, computes $X = [\cos q^T; \sin q^T]$ on the fly (`0.56 us`), and invokes the specialized algebraic screw exponentials (`screw_exp.m`) and hoisted block-diagonal adjoint RNEA kernel (`rnea_shared_kinematics_S1.m`).
4. **Version B (`+panda_s1` — Global Lie-Group Formulation on $(S^1)^7$):**  
   Stores and evolves the configuration natively on the manifold $X = [x_1, \dots, x_7] \in (S^1)^7 \subset \mathbb{R}^{2\times 7}$, evaluates all kinematics and dynamics algebraically without trigonometric calls, and integrates trajectories via Abelian Munthe–Kaas 4-stage Lie-group RK4 (RK4-MK).

---

## 2. Micro-Benchmarks: Isolated Computational Latency

All measurements report the **median execution time per call** in microseconds ($\mu\text{s}$) across $10 \times 100 = 1,000$ evaluations per routine after a 100-pass JIT warm-up.

| Operation | Complexity | Stock MR | Version A ($q \in \mathbb{R}^7$) | Variant B-alg ($q \to S^1$ kernels) | Version B ($X \in (S^1)^7$) | B-alg vs. A (Algorithmic) | B vs. A (Total) | B vs. B-alg (Manifold) |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Forward Kinematics (`fk`, $T_{0e}$)** | $O(n)$ | $43.14\,\mu\text{s}$ | $45.04\,\mu\text{s}$ | $13.80\,\mu\text{s}$ | **$13.14\,\mu\text{s}$** | $3.26\times$ | **$3.43\times$** | $1.05\times$ |
| **Link Frames (`frames`, $T_{0,i}$)** | $O(n)$ | $49.95\,\mu\text{s}$ | $52.39\,\mu\text{s}$ | $8.88\,\mu\text{s}$ | **$7.96\,\mu\text{s}$** | $5.90\times$ | **$6.58\times$** | $1.12\times$ |
| **Space Jacobian ($J_s$)** | $O(n)$ | $47.23\,\mu\text{s}$ | $48.73\,\mu\text{s}$ | $25.23\,\mu\text{s}$ | **$24.53\,\mu\text{s}$** | $1.93\times$ | **$1.99\times$** | $1.03\times$ |
| **Body Jacobian ($J_b$)** | $O(n)$ | $48.66\,\mu\text{s}$ | $97.51\,\mu\text{s}$ | $33.32\,\mu\text{s}$ | **$31.95\,\mu\text{s}$** | $2.93\times$ | **$3.05\times$** | $1.04\times$ |
| **Inverse Dynamics (`id`, RNEA)** | $O(n)$ | $162.86\,\mu\text{s}$ | $103.95\,\mu\text{s}$ | $50.70\,\mu\text{s}$ | **$47.05\,\mu\text{s}$** | $2.05\times$ | **$2.21\times$** | $1.08\times$ |
| **Dynamics (`dyn`, $M$ and $b$)** | $O(n^2) + O(n)$ | $1,272.61\,\mu\text{s}$ | $178.64\,\mu\text{s}$ | $119.50\,\mu\text{s}$ | **$118.69\,\mu\text{s}$** | $1.49\times$ | **$1.51\times$** | $1.01\times$ |
| **Forward Dynamics (`fd`, $\ddot{q}$)** | $O(n^2) + O(n^3)$ | $1,280.04\,\mu\text{s}$ | $178.36\,\mu\text{s}$ | $123.96\,\mu\text{s}$ | **$120.75\,\mu\text{s}$** | $1.44\times$ | **$1.48\times$** | $1.03\times$ |

### Representation Primitives Latency:
- **Chart Lift $q \mapsto X = [\cos q^T; \sin q^T]$ (`panda_s1.from_q`):** $0.56\,\mu\text{s}$
- **Centered Admissible-Arc Recovery $X \mapsto q$ (`panda_s1.to_q`):** $1.05\,\mu\text{s}$
- **Group Action $\operatorname{Exp}_{S^1}(\Delta\theta, X)$ with 1-ulp Normalization (`panda_s1.exp_s1`):** $1.56\,\mu\text{s}$

---

## 3. Critical Decomposition: Algorithmic Specialization vs. Manifold Representation

### 3.1 Where Does the Speedup Come From?
1. **Algorithmic Specialization (`Variant B-alg` vs. `Version A`, $1.44\times \text{–} 5.90\times$ speedup):**  
   The dominant latency reduction ($\sim 90\%\text{–}95\%$ of the total time saved over Version A) comes from two mathematical simplifications enabled by the structure of zero-pitch revolute joints:
   - **Theorem 3.1 (`screw_exp.m`):** Replacing `MatrixExp6(VecTose3(S_i * q_i))` with $\exp([S_i], x_i) = I_4 + s_i[S_i] + (1-c_i)[S_i]^2$.
   - **Adjoint Homomorphism Factorization (`rnea_shared_kinematics_S1.m`):** Precomputing $B^{(i)} = \operatorname{Ad}_{M_i^{-1}}$ and evaluating $\operatorname{Ad}_{T_{i,i-1}(x_i)} = \operatorname{diag}(R_z(x_i)^T, R_z(x_i)^T) B^{(i)}$ by rotating only row pairs $(1,2)$ and $(4,5)$ of $B^{(i)}$, eliminating 7 calls per state to `MatrixExp6`, `VecTose3`, `Adjoint`, `TransToRp`, and `VecToso3` (saving $\sim 57\,\mu\text{s}$ per uncached state).
2. **Direct Manifold State (`Version B` vs. `Variant B-alg`, additional $1.01\times \text{–} 1.12\times$ speedup per operator):**  
   When the configuration is already stored as $X \in (S^1)^7$, individual kinematic and RNEA evaluations avoid even the `from_q` ($\cos q, \sin q$) conversion, saving an additional $0.6\text{–}3.6\,\mu\text{s}$ per call ($5\%\text{–}12\%$ faster than Variant B-alg in `fk`, `frames`, and `inverse_dynamics`).
3. **$O(n)$ Single RNEA vs. $O(n^2)$ Mass Matrix Assembly:**  
   Notice why the relative speedup of Version B over Version A is **$2.21\times$** in `inverse_dynamics` ($103.95\,\mu\text{s} \to 47.05\,\mu\text{s}$, saving $56.9\,\mu\text{s}$), whereas in `dynamics` ($M$ and $b$) it is **$1.51\times$** ($178.64\,\mu\text{s} \to 118.69\,\mu\text{s}$, saving $59.95\,\mu\text{s}$):
   - Both operations share the **exact same $O(n)$ kinematics pass** (`rnea_shared_kinematics`), where Version B saves $\sim 58\,\mu\text{s}$.
   - In `inverse_dynamics`, the remaining $O(n)$ Newton–Euler sweep (`InverseDynamicsSharedKinematics_MR`) takes only $\sim 20\,\mu\text{s}$.
   - In `dynamics`, assembling the $7\times 7$ mass matrix (`MassMatrixZeroStateSharedKinematics_MR`) and bias vector takes an additional $\sim 72\,\mu\text{s}$ that is identical across both representations.

---

## 4. Macro-Benchmark: Closed-Loop 3.6 s Trajectory Simulation

| Simulation Metric | Version A (PoE $q \in \mathbb{R}^7$) | Variant B-alg ($q \in \mathbb{R}^7 + S^1$ Kernels) | Version B (Manifold $X \in (S^1)^7$, RK4-MK) |
|:---|:---:|:---:|:---:|
| **Wall-Clock Simulation Time** | $2,911.14\text{ ms}$ | **$1,973.66\text{ ms}$** | **$2,061.62\text{ ms}$** |
| **Plant Substep Latency** | $808.65\,\mu\text{s}$ | **$548.24\,\mu\text{s}$** | **$572.67\,\mu\text{s}$** |
| **Real-Time Factor (RTF)** | $1.24\times$ | **$1.82\times$** | **$1.75\times$** |
| **Speedup vs. Version A** | $1.00\times$ | **$1.48\times$** | **$1.41\times$** |
| **Nominal Energy Balance Error** | $5.6218 \times 10^{-7}\text{ J}$ | **$9.9790 \times 10^{-11}\text{ J}$** | **$1.0000 \times 10^{-10}\text{ J}$** |
| **Max Trajectory Difference vs. Version B** | $9.7797 \times 10^{-10}\text{ rad}$ | **$4.4409 \times 10^{-15}\text{ rad}$** | $0$ (Reference) |
| **Final Endpoint Difference vs. Version B** | $8.3267 \times 10^{-16}\text{ rad}$ | **$8.8818 \times 10^{-16}\text{ rad}$** | $0$ (Reference) |
| **Max $S^1$ Unit-Norm Error** | N/A | N/A | **$2.2204 \times 10^{-16}$ ($1\,\text{ulp}$)** |
| **Max Joint Limit Violation** | $0.0\text{ rad}$ | $0.0\text{ rad}$ | $0.0\text{ rad}$ |

### Honest Assessment of Simulation Overhead on $(S^1)^7$:
Comparing **Variant B-alg** ($548.24\,\mu\text{s}$/step) and **Version B** ($572.67\,\mu\text{s}$/step) reveals the exact cost of evolving on $(S^1)^7$:
- Both Version B and Variant B-alg are **$>1.41\times$ faster** than Version A ($808.65\,\mu\text{s}$/step).
- Evolving directly on $(S^1)^7$ via Munthe–Kaas RK4-MK (with 4 calls to `exp_s1` per substep, 1-ulp radial normalization, `box_diff` control, and 5-stage $S^1$ limit/norm monitoring) adds only **$24.4\,\mu\text{s}$ per substep ($\sim 4.4\%$ overhead)** relative to unconstrained Euclidean RK4 in $\mathbb{R}^7$ (`Variant B-alg`).

---

## 5. Step-Size Sensitivity & Energy Balance Convergence

| Step Size $h$ | Version A Error (J) | Variant B-alg Error (J) | Version B Error (J) | Ratio (A / B) | Order $p$ (A) | Order $p$ (B-alg) | Order $p$ (B) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **$2.00\text{ ms}$** | $5.6224 \times 10^{-7}\text{ J}$ | $1.9087 \times 10^{-9}\text{ J}$ | **$1.9087 \times 10^{-9}\text{ J}$** | **$294.6\times$** | — | — | — |
| **$1.00\text{ ms}$** | $5.6218 \times 10^{-7}\text{ J}$ | $9.9790 \times 10^{-11}\text{ J}$ | **$1.0000 \times 10^{-10}\text{ J}$** | **$5,621.9\times$** | $0.00$ | $4.26$ | **$4.25$** |
| **$0.50\text{ ms}$** | $5.6217 \times 10^{-7}\text{ J}$ | $5.9242 \times 10^{-12}\text{ J}$ | **$5.7726 \times 10^{-12}\text{ J}$** | **$97,385.9\times$** | $0.00$ | $4.07$ | **$4.11$** |
| **$0.25\text{ ms}$** | $5.6219 \times 10^{-7}\text{ J}$ | $4.5475 \times 10^{-13}\text{ J}$ | **$2.4414 \times 10^{-13}\text{ J}$** | **$2,302,747.1\times$** | $0.00$ | $3.70$ | **$4.56$** |

- **Why Both Variant B-alg and Version B Eliminate the $5.62 \times 10^{-7}\text{ J}$ Error Floor:**  
  Version A calls Modern Robotics `MatrixExp6` $\to$ `MatrixExp3`, which applies `NearZero(norm(omgtheta))` (`|q_i| < 1e-6`), snapping $R_i \to I_3$ and injecting an $O(10^{-6})$ discontinuity at step $k=3$ when joints 1, 3, and 5 accelerate from $q_i(0)=0$. Both Variant B-alg and Version B evaluate $(c_i, s_i)$ smoothly without `NearZero` snapping, recovering theoretical $O(h^4)$ RK4 convergence down to floating-point roundoff ($2.44 \times 10^{-13}\text{ J}$).

---

## 6. Long-Horizon Manifold Stability (5,000 Steps at $\Delta t = 10\text{ ms}$)

| Integrator Scheme on $(S^1)^7$ | Max Unit-Circle Violation $\max_i |x_i^T x_i - 1|$ | Orthogonality Status |
|:---|:---:|:---|
| **Ambient $\mathbb{R}^{14}$ RK4 (Unprojected)** | $7.8731 \times 10^{-9}$ | Systematic $O(h^5)$ radial contraction per step |
| **Lie-Group RK4-MK (Pure $SO(2)$ Rotation, No Renorm)** | $2.1383 \times 10^{-13}$ | Bounded random-walk roundoff |
| **Lie-Group RK4-MK (With 1-ulp Radial Normalization)** | **$2.2204 \times 10^{-16}$** | **Exact to machine precision ($1\,\text{ulp}$), zero phase distortion** |
