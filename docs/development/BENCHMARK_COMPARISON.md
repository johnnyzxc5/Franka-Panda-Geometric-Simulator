# Scientific Benchmark and Comparative Analysis: Stock MR vs. Version A vs. Variant B-alg vs. Version B

**Milestone:** HITO 5, HITO 6 & Revisión R2/R3 — Benchmarks y comparación científica (A / B-alg / B / Stock MR)  
**Date:** October 26, 2026  
**Authors / Models:** Flash 3.8 High & Gemini 4 Argon  
**Environment:** MATLAB R2026a Update 4 (`26.1.0.3312084`), Windows 11, Single-thread warmed CPU  
**Reproducibility Seed:** `13102026` (`twister`), $N = 100$ independent physically admissible states, $R = 10$ runs per state ($1,000$ timed evaluations per routine)  
**Execution Tool:** `tools/benchmark_s1_comparison.m`  
**Output Artifacts:** `results/benchmark_s1_comparison.json`, `results/benchmark_s1_comparison.mat`, `results/benchmark_s1_comparison.png`, and `docs/assets/benchmark_s1_comparison.png`

---

## 1. Experimental Design: Four-Way Scientific Decomposition & Physically Admissible Sampling

To rigorously separate **algorithmic specialization** (replacing generic 6D matrix exponentials with specialized revolute rotations and hoisted adjoints) from the **global manifold representation** ($X \in (S^1)^7$ vs. $q \in \mathbb{R}^7$), `tools/benchmark_s1_comparison.m` evaluates four implementations on identical physical states:

1. **Stock Modern Robotics (MR Textbook Baseline):**  
   Standard `MatrixExp6`, `JacobianSpace`, `JacobianBody`, `InverseDynamics`, and `MassMatrix` from Lynch & Park (2017).
2. **Version A (`+panda` — Original Optimized Euclidean Formulation):**  
   Joint coordinates $q \in \mathbb{R}^7$, `MatrixExp6` PoE, and the SHAPE RNEA kernel (`rnea_shared_kinematics_MR.m`, zero-state mass recursion, hoisted constant transforms).
3. **Variant B-alg (`q` + Specialized Algebraic Kernels — Control Benchmark):**  
   Retains Euclidean coordinates $q \in \mathbb{R}^7$ as input and integration state, computes $X = [\cos q^\top; \sin q^\top]$ on the fly (`0.59 us`), and invokes the specialized algebraic screw exponentials (`screw_exp.m`) and hoisted block-diagonal adjoint RNEA kernel (`rnea_shared_kinematics_S1.m`).
4. **Version B (`+panda_s1` — Global Lie-Group Formulation on $(S^1)^7$):**  
   Stores and evolves the configuration natively on the manifold $X = [x_1, \dots, x_7] \in (S^1)^7 \subset \mathbb{R}^{2\times 7}$, evaluates all kinematics and inverse dynamics algebraically without trigonometric calls, and integrates trajectories via Abelian Munthe–Kaas 4-stage Lie-group RK4 (RK4-MK).

### Physically Admissible Operating Envelope Verification (`sampling` in JSON):
All $N = 100$ benchmark states are sampled strictly inside the physical limits of the Franka Emika Panda via bounded uniform sampling for positions and rejection-truncated normal sampling for velocities, accelerations, and torques:
- **Joint Position Utilization:** $\max_{i,k} |q_{i,k} - m_i|/h_i = 0.9686 \le 1.00$ (`0` violations).
- **Joint Velocity Utilization:** $\max_{i,k} |\dot{q}_{i,k}|/\dot{q}_{\max,i} = 0.9459 \le 0.95$ (`0` violations).
- **Joint Torque Utilization:** $\max_{i,k} |\tau_{i,k}|/\tau_{\max,i} = 0.9277 \le 0.95$ (`0` violations).
- **Joint Acceleration Bound:** $\max_{i,k} |\ddot{q}_{i,k}| = 7.1162\text{ rad/s}^2 \le 10.0\text{ rad/s}^2$.

---

## 2. Micro-Benchmarks: Isolated Computational Latency

All measurements report the **median execution time per call** in microseconds ($\mu\text{s}$) across $10 \times 100 = 1,000$ evaluations per routine after a 100-pass JIT warm-up (`results/benchmark_s1_comparison.json`).

| Operation | Complexity | Stock MR | Version A ($q \in \mathbb{R}^7$) | Variant B-alg ($q \to S^1$ kernels) | Version B ($X \in (S^1)^7$) | B-alg vs. A (Algorithmic) | B vs. A (Total) | B vs. B-alg (Manifold) |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Forward Kinematics (`fk`, $T_{0e}$)** | $O(n)$ | $42.13\,\mu\text{s}$ | $42.24\,\mu\text{s}$ | $14.92\,\mu\text{s}$ | **$13.29\,\mu\text{s}$** | $2.83\times$ | **$3.18\times$** | $1.12\times$ |
| **Link Frames (`frames`, $T_{0,i}$)** | $O(n)$ | $49.28\,\mu\text{s}$ | $52.49\,\mu\text{s}$ | $9.96\,\mu\text{s}$ | **$9.08\,\mu\text{s}$** | $5.27\times$ | **$5.78\times$** | $1.10\times$ |
| **Space Jacobian ($J_s$)** | $O(n)$ | $48.10\,\mu\text{s}$ | $49.01\,\mu\text{s}$ | $25.90\,\mu\text{s}$ | **$25.24\,\mu\text{s}$** | $1.89\times$ | **$1.94\times$** | $1.03\times$ |
| **Body Jacobian ($J_b$)** | $O(n)$ | $48.07\,\mu\text{s}$ | $96.25\,\mu\text{s}$ | $33.11\,\mu\text{s}$ | **$31.73\,\mu\text{s}$** | $2.91\times$ | **$3.03\times$** | $1.04\times$ |
| **Inverse Dynamics (`id`, RNEA)** | $O(n)$ | $162.38\,\mu\text{s}$ | $105.27\,\mu\text{s}$ | $51.45\,\mu\text{s}$ | **$49.07\,\mu\text{s}$** | $2.05\times$ | **$2.15\times$** | $1.05\times$ |
| **Dynamics (`dyn`, $M$ and $b$)** | $O(n^2) + O(n)$ | $1,271.50\,\mu\text{s}$ | $178.47\,\mu\text{s}$ | **$124.34\,\mu\text{s}$** | $124.99\,\mu\text{s}$ | $1.44\times$ | **$1.43\times$** | $0.99\times$ |
| **Forward Dynamics (`fd`, $\ddot{q}$)** | $O(n^2) + O(n^3)$ | $1,297.35\,\mu\text{s}$ | $186.53\,\mu\text{s}$ | $127.72\,\mu\text{s}$ | **$124.53\,\mu\text{s}$** | $1.46\times$ | **$1.50\times$** | $1.03\times$ |

### Representation Primitives Latency:
- **Chart Embedding $q \mapsto X = [\cos q^\top; \sin q^\top]$ (`panda_s1.from_q`):** $0.59\,\mu\text{s}$
- **Centered Admissible-Arc Recovery $X \mapsto q$ (`panda_s1.to_q`):** $1.49\,\mu\text{s}$
- **Group Action $\operatorname{Exp}_{S^1}(\Delta\theta, X)$ with 1-ulp Normalization (`panda_s1.exp_s1`):** $2.01\,\mu\text{s}$

---

## 3. Critical Decomposition: Algorithmic Specialization vs. Manifold Representation

### 3.1 Where Does the Speedup Come From?
1. **Algorithmic Specialization (`Variant B-alg` vs. `Version A`, $1.44\times \text{–} 5.27\times$ speedup):**  
   The dominant latency reduction ($\sim 88\%\text{–}95\%$ of the total time saved over Version A) comes from two mathematical simplifications enabled by the structure of unit-axis, zero-pitch revolute joints:
   - **Theorem 3.1 (`screw_exp.m`):** Replacing `MatrixExp6(VecTose3(S_i * q_i))` with $\exp([S_i], x_i) = I_4 + s_i[S_i] + (1-c_i)[S_i]^2$.
   - **Adjoint Homomorphism Factorization (`rnea_shared_kinematics_S1.m`):** Precomputing $B^{(i)} = \operatorname{Ad}_{M_i^{-1}}$ and evaluating $\operatorname{Ad}_{T_{i,i-1}(x_i)} = \operatorname{diag}(R_z(x_i)^\top, R_z(x_i)^\top) B^{(i)}$ by rotating only row pairs $(1,2)$ and $(4,5)$ of $B^{(i)}$, eliminating 7 calls per state to `MatrixExp6`, `VecTose3`, `Adjoint`, `TransToRp`, and `VecToso3`.
2. **Direct Manifold State (`Version B` vs. `Variant B-alg`, up to $1.12\times$ additional speedup in kinematics):**  
   When the configuration is already stored as $X \in (S^1)^7$, individual kinematic and RNEA evaluations avoid the `from_q` ($\cos q, \sin q$) conversion, yielding an additional $3\%\text{–}12\%$ speedup in `fk`, `frames`, `jacobian`, `inverse_dynamics`, and `forward_dynamics`, while `dynamics` ($M$ and $b$) is effectively tied ($124.34\,\mu\text{s}$ vs. $124.99\,\mu\text{s}$).

---

## 4. Macro-Benchmark: Closed-Loop 3.6 s Trajectory Simulation

| Simulation Metric | Version A (PoE $q \in \mathbb{R}^7$) | Variant B-alg ($q \in \mathbb{R}^7 + S^1$ Kernels) | Version B (Manifold $X \in (S^1)^7$, RK4-MK) |
|:---|:---:|:---:|:---:|
| **Wall-Clock Simulation Time** | $3,067.88\text{ ms}$ | **$2,091.79\text{ ms}$** | **$2,158.82\text{ ms}$** |
| **Plant Substep Latency** | $852.19\,\mu\text{s}$ | **$581.05\,\mu\text{s}$** | **$599.67\,\mu\text{s}$** |
| **Real-Time Factor (RTF)** | $1.17\times$ | **$1.72\times$** | **$1.67\times$** |
| **Speedup vs. Version A** | $1.00\times$ | **$1.47\times$** | **$1.42\times$** |
| **Nominal Energy Balance Error** | $5.6218 \times 10^{-7}\text{ J}$ | **$9.9790 \times 10^{-11}\text{ J}$** | **$1.0000 \times 10^{-10}\text{ J}$** |
| **Max Trajectory Difference vs. Version B** | $9.7797 \times 10^{-10}\text{ rad}$ | **$4.4409 \times 10^{-15}\text{ rad}$** | $0$ (Reference) |
| **Final Endpoint Difference vs. Version B** | $8.3267 \times 10^{-16}\text{ rad}$ | **$8.8818 \times 10^{-16}\text{ rad}$** | $0$ (Reference) |
| **Max $S^1$ Unit-Norm Error** | N/A | N/A | **$2.2204 \times 10^{-16}$ ($1\,\text{ulp}$)** |
| **Max Joint Limit Violation** | $0.0\text{ rad}$ | $0.0\text{ rad}$ | $0.0\text{ rad}$ |

### Honest Assessment of Simulation Overhead on $(S^1)^7$:
Comparing **Variant B-alg** ($581.05\,\mu\text{s}$/step) and **Version B** ($599.67\,\mu\text{s}$/step) reveals the exact cost of evolving on $(S^1)^7$:
- Both Version B ($1.42\times$) and Variant B-alg ($1.47\times$) are significantly faster than Version A ($852.19\,\mu\text{s}$/step).
- Evolving directly on $(S^1)^7$ via Munthe–Kaas RK4-MK (with 4 calls to `exp_s1` per substep, 1-ulp radial normalization, `box_diff` control, and 5-stage $S^1$ limit/norm monitoring) adds only **$18.6\,\mu\text{s}$ per substep ($\sim 3.2\%$ overhead)** relative to unconstrained Euclidean RK4 in $\mathbb{R}^7$ (`Variant B-alg`).

---

## 5. Step-Size Sensitivity & Energy Balance Convergence

| Step Size $h$ | Version A Error (J) | Variant B-alg Error (J) | Version B Error (J) | Ratio (A / B) | Order $p$ (A) | Order $p$ (B-alg) | Order $p$ (B) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **$2.00\text{ ms}$** | $5.6224 \times 10^{-7}\text{ J}$ | $1.9087 \times 10^{-9}\text{ J}$ | **$1.9087 \times 10^{-9}\text{ J}$** | **$294.6\times$** | — | — | — |
| **$1.00\text{ ms}$** | $5.6218 \times 10^{-7}\text{ J}$ | $9.9790 \times 10^{-11}\text{ J}$ | **$1.0000 \times 10^{-10}\text{ J}$** | **$5,621.9\times$** | $0.00$ | $4.26$ | **$4.25$** |
| **$0.50\text{ ms}$** | $5.6217 \times 10^{-7}\text{ J}$ | $5.8794 \times 10^{-12}\text{ J}$ | **$5.7726 \times 10^{-12}\text{ J}$** | **$97,385.9\times$** | $0.00$ | $4.09$ | **$4.11$** |
| **$0.25\text{ ms}$** | $5.6219 \times 10^{-7}\text{ J}$ | $4.2916 \times 10^{-13}\text{ J}$ | **$2.4414 \times 10^{-13}\text{ J}$** | **$2,302,747.1\times$** | $0.00$ | $3.78$ | **$4.56$** |

- **Why Both Variant B-alg and Version B Eliminate the $5.62 \times 10^{-7}\text{ J}$ Error Floor:**  
  Version A calls Modern Robotics `MatrixExp6` $\to$ `MatrixExp3`, which applies `NearZero(norm(omgtheta))` (`|q_i| < 1e-6`), snapping $R_i \to I_3$ and injecting an $O(10^{-6})$ discontinuity at step $k=3$ when joints 1, 3, and 5 accelerate from $q_i(0)=0$. Both Variant B-alg and Version B evaluate $(c_i, s_i)$ smoothly without `NearZero` snapping, recovering theoretical $O(h^4)$ RK4 convergence down to floating-point roundoff ($2.44 \times 10^{-13}\text{ J}$).

---

## 6. Long-Horizon Manifold Stability (5,000 Steps at $\Delta t = 10\text{ ms}$)

| Integrator Scheme on $(S^1)^7$ | Max Unit-Circle Violation $\max_i |x_i^\top x_i - 1|$ | Orthogonality Status |
|:---|:---:|:---|
| **Ambient $\mathbb{R}^{14}$ RK4 (Unprojected)** | $7.8731 \times 10^{-9}$ | Systematic $O(h^5)$ radial contraction per step |
| **Lie-Group RK4-MK (Pure $SO(2)$ Rotation, No Renorm)** | $2.1383 \times 10^{-13}$ | Bounded random-walk roundoff |
| **Lie-Group RK4-MK (With 1-ulp Radial Normalization)** | **$2.2204 \times 10^{-16}$** | **Exact to machine precision ($1\,\text{ulp}$), zero phase distortion** |
