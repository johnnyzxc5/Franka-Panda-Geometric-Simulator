# Geometric & Dynamic Implementation on $(S^1)^7$

**Milestone:** HITO 3 — Implementación geométrica y dinámica  
**Date:** October 26, 2026  
**Author / Model:** Argon (Gemini 4 Argon)

---

## 1. Overview of the Implemented Geometric Core

In Milestone 3, the global $(S^1)^7$ kinematic and dynamic core (`+panda_s1/` and `third_party/shape_kernels/rnea_shared_kinematics_S1.m`) has been completed so that every kinematic and dynamic operator evaluates **intrinsically and algebraically on $(S^1)^7$**, eliminating intermediate angle conversions (`to_q` / `atan2`) and transcendental calls (`cos`, `sin`, `MatrixExp6`) on all nominal revolute joints while preserving exact fallback compatibility for arbitrary screw perturbations.

---

## 2. Transformations, Conventions, and Algebraic Operators

### 2.1 Algebraic Space Screw Exponential (`+panda_s1/screw_exp.m`)

For a screw axis $S_i = (\hat{\omega}_i, v_i) \in \mathbb{R}^6$ with $\|\hat{\omega}_i\|_2 = 1$ and $x_i = (c_i, s_i)^T \in S^1$, let $\Omega_i = [\hat{\omega}_i]_\times \in \mathfrak{so}(3)$ and $h_{\text{pitch},i} = \hat{\omega}_i^T v_i$. By Rodrigues' formula and the identity $\Omega_i^3 = -\Omega_i$, the $SE(3)$ exponential $\exp([S_i] q_i)$ splits into a purely polynomial part in $(c_i, s_i)$ plus an axial pitch translation:
$$\exp([S_i], x_i) = \begin{bmatrix} I_3 + s_i \Omega_i + (1 - c_i)\Omega_i^2 & s_i v_i + (1 - c_i)(\Omega_i v_i) + (\varpi_i(x_i) - s_i) h_{\text{pitch},i} \hat{\omega}_i \\ 0_{1\times 3} & 1 \end{bmatrix}.$$
- **Pure Revolute Joints ($h_{\text{pitch},i} = 0$, all 7 nominal Panda joints):** The axial pitch term vanishes identically. Therefore, `panda_s1.screw_exp` evaluates $\exp([S_i], x_i)$ using **only multiplications and additions** of $c_i$ and $s_i$, with zero calls to `atan2`, `cos`, or `sin`.
- **Perturbed Helical Screws ($|h_{\text{pitch},i}| > 10^{-14}$ in test suites):** When a test artificially perturbs `Slist` (e.g., `altered.Slist(6,2) = altered.Slist(6,2) + 0.01`), `panda_s1.screw_exp` automatically activates the centered lift $\varpi_i(x_i) = m_i + \operatorname{atan2}(s_i \cos m_i - c_i \sin m_i, c_i \cos m_i + s_i \sin m_i)$, remaining branch-cut-safe even when Joint 6 exceeds $\pi$.

### 2.2 Independent Forward Kinematics Paths (`fk.m` vs. `frames.m`)

To preserve the mathematical independence between the Space Product of Exponentials and the Relative Body Frame Chain:
1. **`+panda_s1/fk.m` (Space PoE on $(S^1)^7$):**
   $$T(X) = \exp([S_1], x_1) \exp([S_2], x_2) \cdots \exp([S_7], x_7) M_{\text{home}},$$
   evaluated via `panda_s1.screw_exp`.
2. **`+panda_s1/frames.m` (Relative Link Chain on $(S^1)^7$):**
   $$T_{0,i}(X) = T_{0,i-1}(X) \, M_i \, T_z(x_i), \qquad i = 1,\dots,7, \qquad T_{0,8}(X) = T_{0,7}(X) M_8,$$
   where post-multiplication by $T_z(x_i)$ rotates columns 1 and 2 of $T_{0,i-1}(X) M_i$ directly by $(c_i, s_i)$:
   $$T(:, 1:2) \leftarrow \begin{bmatrix} c_i T(:,1) + s_i T(:,2), & -s_i T(:,1) + c_i T(:,2) \end{bmatrix}.$$

### 2.3 Algebraic Space and Body Twist Jacobians (`+panda_s1/jacobian.m`)

`+panda_s1/jacobian.m` computes the $6\times 7$ space Jacobian $J_s(X)$ by accumulating $T_{i-1}(X) = \prod_{k=1}^{i-1} \exp([S_k], x_k)$ via `panda_s1.screw_exp` and projecting $J_{s,i}(X) = \operatorname{Ad}_{T_{i-1}(X)} S_i$. When `'body'` frame is requested, it reuses the already accumulated product $T_6(X) = \prod_{k=1}^6 \exp([S_k], x_k)$ to obtain $T_{\text{flange}}(X) = T_6(X) \exp([S_7], x_7) M_{\text{home}}$ in a single additional step and transforms $J_b(X) = \operatorname{Ad}_{T_{\text{flange}}(X)^{-1}} J_s(X)$.

---

## 3. Adjoint Homomorphism Optimization in `rnea_shared_kinematics_S1.m`

### 3.1 Exact Block-Diagonal Adjoint Factorization

In the original kernel `rnea_shared_kinematics_MR.m`, each uncached evaluation computes:
$$\operatorname{Ad}_{T_{i,i-1}(q_i)} = \operatorname{Ad}\!\left(\exp(-[A_i] q_i) \, M_i^{-1}\right), \qquad i = 1,\dots,7,$$
which invokes `VecTose3`, `MatrixExp6` (with `cos`, `sin`, `norm`), $4\times 4$ matrix multiplication, `Adjoint`, `TransToRp`, and `VecToso3` at every joint.

For all 7 nominal joints of the Franka Panda, the local screw axis in link frame $i$ is $A_i = e_3 = [0, 0, 1, 0, 0, 0]^T$ (to within $1.32 \times 10^{-33}$). Therefore, $\exp(-[A_i] q_i) = \operatorname{diag}(R_z(x_i)^T, 1)$ has zero translation, and its $6\times 6$ Adjoint matrix is block-diagonal:
$$\operatorname{Ad}_{\exp(-[A_i] q_i)} = \begin{bmatrix} R_z(x_i)^T & 0_{3\times 3} \\ 0_{3\times 3} & R_z(x_i)^T \end{bmatrix}, \qquad R_z(x_i)^T = \begin{bmatrix} c_i & s_i & 0 \\ -s_i & c_i & 0 \\ 0 & 0 & 1 \end{bmatrix}.$$
By the group homomorphism property $\operatorname{Ad}_{T_1 T_2} = \operatorname{Ad}_{T_1} \operatorname{Ad}_{T_2}$:
$$\boxed{\operatorname{Ad}_{T_{i,i-1}(x_i)} = \begin{bmatrix} R_z(x_i)^T & 0_{3\times 3} \\ 0_{3\times 3} & R_z(x_i)^T \end{bmatrix} B^{(i)}, \qquad B^{(i)} := \operatorname{Ad}_{M_i^{-1}} \in \mathbb{R}^{6\times 6}.}$$

### 3.2 Computational Impact

During invariant hoisting (`need_rebuild == true`), `rnea_shared_kinematics_S1.m` precomputes and caches the constant $6\times 6$ matrices $B^{(i)} = \operatorname{Ad}_{M_i^{-1}}$ (`cached_Ad_invM`). At runtime, for each joint $i=1,\dots,7$, computing $\operatorname{Ad}_{T_{i,i-1}(x_i)}$ reduces to rotating row pairs $(1,2)$ and $(4,5)$ of $B^{(i)}$ by $(c_i, s_i)$ while copying rows $3$ and $6$:
```matlab
B = cached_Ad_invM(:, :, i);
AdTi(:, :, i) = [ ci * B(1, :) + si * B(2, :); ...
                 -si * B(1, :) + ci * B(2, :); ...
                  B(3, :); ...
                  ci * B(4, :) + si * B(5, :); ...
                 -si * B(4, :) + ci * B(5, :); ...
                  B(6, :) ];
```
- **Zero function calls** inside the joint loop (`MatrixExp6`, `VecTose3`, `Adjoint`, `TransToRp`, `VecToso3` are completely eliminated).
- **Exact floating-point parity:** Maximum Frobenius discrepancy against `rnea_shared_kinematics_MR` across 700 joint evaluations is $2.38 \times 10^{-16}$ ($1\,\text{ulp}$).
- **Automatic Fallback for Perturbed Geometries:** If `Mlist` or `Slist` is modified in a test such that $\|A_i - e_3\|_\infty \ge 10^{-12}$, `cached_is_pure_z(i)` becomes `false` and the kernel automatically invokes `panda_s1.screw_exp` with centered arc midpoints, passing all cache-invalidation and screw-perturbation tests.
- **Bitwise State Cache:** Uses `typecast(X(:), 'uint64')` ($14\times 1$ `uint64`) to provide exact state-cache hits between successive calls (`dynamics`, `inverse_dynamics`, `energy`) at the same configuration $X \in (S^1)^7$.

---

## 4. Independent Lagrange Oracle and Tangential Calculus on $(S^1)^7$

1. **`+panda_s1/lagrange_reference.m`:**  
   Computes the mass matrix $M_L(X)$, gravity vector $g_L(X)$, and potential energy $V_L(X)$ directly from the algebraic link frames `panda_s1.frames(X, m)` and explicit center-of-mass Jacobians $J_{v,i}(X), J_{\omega,i}(X)$, without calling any RNEA kernel.
2. **Intrinsic Tangential Finite Differences on $(S^1)^7$ (`tests/validate_panda_s1.m`):**  
   Using the $SO(2)^7$ exponential action $X_{\pm}^{(j)} = \operatorname{Exp}_{S^1}(\pm \epsilon e_j, X)$, the tangential directional derivative $\mathcal{D}_j \Phi(X) = (J x_j)^T \frac{\partial \Phi}{\partial x_j}$ of Lee et al. (Chapter 4, Proposition 4.2) is approximated to $O(\epsilon^2)$ directly on the manifold:
   $$\mathcal{D}_j \Phi(X) \approx \frac{\Phi(\operatorname{Exp}_{S^1}(\epsilon e_j, X)) - \Phi(\operatorname{Exp}_{S^1}(-\epsilon e_j, X))}{2\epsilon},$$
   verifying the Christoffel Coriolis vector $c(X,\omega)$, the potential gradient $g(X) = \mathcal{D} V(X)$, the skew-symmetry power identity $\omega^T c(X,\omega) = \frac{1}{2} \omega^T \dot{M}(X,\omega)\omega$, and the space twist Jacobian $J_s(X)$ completely on $(S^1)^7$.
