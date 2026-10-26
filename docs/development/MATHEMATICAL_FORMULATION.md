# Mathematical Formulation: Global Geometric Dynamics of the Franka Panda on $(S^1)^7$

**Milestone:** HITO 1 — Diseño matemático de la representación global  
**Date:** October 26, 2026  
**Author / Model:** Argon (Gemini 4 Argon)  
**Primary References:**  
1. T. Lee, M. Leok, and N. H. McClamroch, *Global Formulations of Lagrangian and Hamiltonian Dynamics on Manifolds*, Springer, 2018 (Chapters 1, 4, and Section 8.7.4.1).  
2. C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, and A. De Luca, "Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization," *IEEE RA-L*, 4(4):4147–4154, 2019.  
3. J. Carpentier et al., *Pinocchio: Fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives* (`JointModelRevoluteUnbounded`).

---

## 1. Executive Summary and Architectural Synthesis

This document establishes the mathematical formulation for **Version B (Global Lie-Group / Manifold Geometric Dynamics)** of the 7-DoF Franka Emika Panda simulator, alongside the intermediate **Variant B-alg (Angular Specialized $q$-Formulation)** required for the three-way scientific benchmark in Milestone 5.

### Key Mathematical Findings

1. **Optimal Global Configuration Manifold:**  
   Each revolute joint $i \in \{1,\dots,7\}$ has intrinsic configuration space $S^1 \subset \mathbb{R}^2$, represented by the unit vector $x_i = (c_i, s_i)^T = (\cos q_i, \sin q_i)^T$ satisfying $x_i^T x_i = 1$. The global configuration manifold of the 7-DoF arm is the 7-torus embedded in $\mathbb{R}^{14}$ (stored canonically as a $2 \times 7$ matrix $X = [x_1, \dots, x_7] \in (S^1)^7 \subset \mathbb{R}^{2\times 7}$):
   $$\mathsf{Q} = (S^1)^7 = \{ X = [x_1, \dots, x_7] \in \mathbb{R}^{2\times 7} : x_i^T x_i = 1, \; i=1,\dots,7 \}.$$
   By the Lie group isomorphism $S^1 \cong SO(2)$, each $x_i$ uniquely defines a planar rotation matrix $R_i(x_i) = [x_i, J x_i] \in SO(2)$ without evaluating transcendental functions (`cos`, `sin`). Independent link poses on $SE(3)^7$ (84 coordinates with 35 holonomic constraints) are rejected because they introduce severe algebraic redundancy and destroy the $O(n)$ tree structure of recursive dynamics.

2. **Exact Equivalence of Lee's Reduced Equations on $T(S^1)^7$ and Spatial RNEA:**  
   In Lee, Leok & McClamroch (Chapter 4, Proposition 4.2, Eq. 4.12; Section 8.7.4.1, Eq. 8.70), the tangent bundle $T(S^1)^n$ is globally trivialized as $(S^1)^n \times \mathbb{R}^n$ via the kinematic relation $\dot{x}_i = \omega_i J x_i$, where $\omega_i \in \mathbb{R}$ is the physical joint angular velocity. Projecting the ambient Euler–Lagrange equations onto $T_x(S^1)^n$ along the tangent basis $J x_i$ eliminates the normal Lagrange multipliers $\lambda_i x_i$ (centripetal constraint forces $-m_{ii}\|\dot{x}_i\|^2 x_i$) and yields the $n$ scalar equations:
   $$\frac{d}{dt}\left(\frac{\partial \tilde{L}(X,\omega)}{\partial \omega_i}\right) + x_i^T J \frac{\partial \tilde{L}(X,\omega)}{\partial x_i} = \tau_i, \qquad i = 1,\dots,n.$$
   We prove in Section 4 that the directional derivative operator $-x_i^T J \frac{\partial}{\partial x_i} = (J x_i)^T \frac{\partial}{\partial x_i}$ is the global pushforward of $\frac{\partial}{\partial q_i}$. Consequently, the reduced Euler–Lagrange equations on $(S^1)^7 \times \mathbb{R}^7$ produce the **exact same** mass matrix $M(X)$, Coriolis/centrifugal vector $c(X,\omega)$, and gravity vector $g(X)$ as the classical formulation, evaluated directly as polynomial/rational functions of $X \in (S^1)^7$. Furthermore, the spatial Recursive Newton–Euler Algorithm (RNEA) depends on the configuration **exclusively** through the inter-link Adjoint transformations $\operatorname{Ad}_{T_{i,i-1}(x_i)}$, allowing full reuse of the velocity/acceleration/wrench recursion kernels.

3. **Critical Discovery on Joint Limits and Geodesic vs. Admissible Differences:**  
   All 7 Panda joints have angular span $2h_i = q_{\max,i} - q_{\min,i} < 2\pi$, so the admissible set $\mathcal{A}_i \subset S^1$ is a proper closed circular arc characterized algebraically by $u_i^T x_i \ge \cos h_i$. However, **6 out of the 7 Panda joints (Joints 1, 2, 3, 5, 6, 7) have admissible span $2h_i > \pi$**. For these 6 joints, the *forbidden complement arc* $S^1 \setminus \mathcal{A}_i$ has length $2\pi - 2h_i < \pi$—meaning the shortest geodesic path on $S^1$ between two configurations near opposite joint limits passes **directly through the physical hard stop**!  
   Therefore, while intrinsic $S^1$ chordal/geodesic errors ($x_i^T J^T x_{\text{ref},i}$ or $\operatorname{Log}_{SO(2)}$) are appropriate for *unbounded* revolute joints, a bounded manipulator **must** use the centered admissible-arc lift $\varpi_i: \mathcal{A}_i \to [q_{\min,i}, q_{\max,i}]$ for large-displacement control and trajectory generation so that the branch cut lies at $m_i \pm \pi \in S^1 \setminus \mathcal{A}_i$.

---

## 2. Comparison of Configuration Representations

We compare four candidate mathematical representations for the 7-DoF Panda arm:

| Representation | Configuration State | Dimension / Storage | Advantages | Disadvantages / Risks |
|---|---|:---:|---|---|
| **1. Euclidean Angles** (Version A) | $q \in \mathbb{R}^7$ | $7$ doubles | Minimal dimension; interval bounds $q_{\min} \le q \le q_{\max}$ are simple box constraints; trivial addition $q + \Delta q$. | Requires $7$ `cos`/`sin` evaluations per kinematics call; not globally isomorphic to $(S^1)^7$ if joints are unbounded (wrapping discontinuities at $\pm\pi$). |
| **2. Unit Circle Embedding** (Selected for Version B) | $X = [x_1,\dots,x_7] \in (S^1)^7$, $x_i = (c_i,s_i)^T \in \mathbb{R}^2$ | $14$ doubles ($2 \times 7$) | Globally singularity-free on $(S^1)^7$; poses, Jacobians, and RNEA Adjoints are purely algebraic (quadratic/linear) in $(c_i,s_i)$; matches Lee Ch. 4 and Pinocchio `JointModelRevoluteUnbounded`. | Requires $c_i^2+s_i^2=1$ preservation during integration; geodesic log on $S^1$ can cross forbidden stops when $2h_i > \pi$. |
| **3. Explicit $SO(2)$ Matrices** | $R_i = \begin{bmatrix} c_i & -s_i \\ s_i & c_i \end{bmatrix} \in SO(2)$ | $28$ doubles ($2 \times 2 \times 7$) | Group multiplication is standard $2\times 2$ matrix multiplication. | Stores each $(c_i,s_i)$ twice ($2\times$ memory and cache footprint of $S^1$) with zero extra information since $R_i = [x_i, J x_i]$. |
| **4. Maximal $SE(3)^7$ Link Poses** | $(R_{\text{link},i}, p_{\text{link},i}) \in SE(3)^7$ | $84$ doubles ($12 \times 7$) | Gravity potential $V$ is strictly linear in $p_{\text{link},i}$. | Requires $35$ bilateral holonomic joint constraints (DAE index 3); destroys $O(n)$ tree recursion; $12\times$ state size. Explicitly excluded by project scope. |

### Justification of Choice: $X \in (S^1)^7$ with $SO(2)$ Action

The unit-circle product manifold $\mathsf{Q} = (S^1)^7$ represented as $X = [x_1, \dots, x_7] \in \mathbb{R}^{2\times 7}$ with $x_i = (c_i, s_i)^T$ is canonically isomorphic to $(SO(2))^7$ via the map:
$$\Phi: S^1 \xrightarrow{\sim} SO(2), \qquad \Phi(x_i) = \begin{bmatrix} x_i & J x_i \end{bmatrix} = \begin{bmatrix} c_i & -s_i \\ s_i & c_i \end{bmatrix}, \qquad J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}.$$
Notice that $J^2 = -I_2$, $J^T = -J$, and for any $x_i, y_i \in S^1$:
- **Group product (angle addition):** $\Phi(x_i) y_i = c_i y_i + s_i J y_i \in S^1$.
- **Group inverse (angle negation):** $\Phi(x_i)^T e_1 = (c_i, -s_i)^T = \operatorname{diag}(1,-1) x_i \in S^1$.
- **Relative rotation (angle difference $q_2 - q_1$):** $\Phi(x_1)^T x_2 = \begin{bmatrix} x_1^T x_2 \\ (J x_1)^T x_2 \end{bmatrix} = \begin{bmatrix} x_1^T x_2 \\ x_1^T J^T x_2 \end{bmatrix} = \begin{bmatrix} \cos(q_2 - q_1) \\ \sin(q_2 - q_1) \end{bmatrix} \in S^1$.

Thus, storing $X \in \mathbb{R}^{2\times 7}$ captures the full Lie-group structure of $(SO(2))^7$ and the homogeneous-manifold structure of Lee Chapter 4 with half the memory of $2\times 2\times 7$ rotation matrices.

---

## 3. Global Kinematics on $(S^1)^7$ and Spatial $SE(3)$ Connection

### 3.1 Tangent Bundle Trivialization and Angular Velocity

Following Lee, Leok & McClamroch (Section 4.2, Eq. 4.1), any smooth curve $t \mapsto x_i(t) \in S^1$ satisfies $x_i(t)^T \dot{x}_i(t) = 0$. Since $\{x_i, J x_i\}$ is a positively oriented orthonormal frame of $\mathbb{R}^2$ at every $x_i \in S^1$, the tangent space at $x_i$ is:
$$T_{x_i} S^1 = \{ \xi_i \in \mathbb{R}^2 : x_i^T \xi_i = 0 \} = \operatorname{span}(J x_i).$$
Therefore, there exists a unique scalar angular velocity $\omega_i(t) \in \mathbb{R}$ such that:
$$\boxed{\dot{x}_i = \omega_i J x_i = \begin{bmatrix} -\omega_i s_i \\ \omega_i c_i \end{bmatrix}, \qquad \omega_i = (J x_i)^T \dot{x}_i = -x_i^T J \dot{x}_i = c_i \dot{s}_i - s_i \dot{c}_i.}$$
This defines a **global diffeomorphism** (left trivialization of the Lie group $(SO(2))^7$):
$$\Psi: T(S^1)^7 \xrightarrow{\sim} (S^1)^7 \times \mathbb{R}^7, \qquad (X, \dot{X}) \mapsto (X, \omega).$$
Differentiating $\dot{x}_i = \omega_i J x_i$ once more gives the ambient second derivative on $S^1$:
$$\ddot{x}_i = \dot{\omega}_i J x_i + \omega_i J \dot{x}_i = \dot{\omega}_i J x_i + \omega_i^2 J^2 x_i = \dot{\omega}_i J x_i - \omega_i^2 x_i,$$
where $\dot{\omega}_i J x_i \in T_{x_i} S^1$ is the tangential angular acceleration and $-\omega_i^2 x_i \in N_{x_i} S^1$ is the normal centripetal acceleration required to remain on the unit circle.

### 3.2 Direct Algebraic Embedding into $SO(3)$ and $SE(3)$

In the Franka Panda model (`+panda/model.m`), each joint $i \in \{1,\dots,7\}$ is a revolute joint whose local axis in body frame $i$ is $a_i = [0, 0, 1]^T$, with local body screw $A_i = \operatorname{Ad}_{(M_1\cdots M_i)^{-1}} S_i = [0, 0, 1, 0, 0, 0]^T$ (verified numerically to within $1.32 \times 10^{-33}$ in MATLAB).

1. **Local Joint Transformation ($z$-axis rotation):**  
   Given $x_i = (c_i, s_i)^T \in S^1$, the local 3D rotation and homogeneous transformation are purely algebraic:
   $$R_z(x_i) = \begin{bmatrix} c_i & -s_i & 0 \\ s_i & c_i & 0 \\ 0 & 0 & 1 \end{bmatrix} \in SO(3), \qquad T_z(x_i) = \begin{bmatrix} R_z(x_i) & 0_{3\times 1} \\ 0_{1\times 3} & 1 \end{bmatrix} \in SE(3).$$
   Its inverse is simply the transpose $T_z(x_i)^{-1} = T_z(\bar{x}_i)$, where $\bar{x}_i = (c_i, -s_i)^T$:
   $$R_z(x_i)^T = \begin{bmatrix} c_i & s_i & 0 \\ -s_i & c_i & 0 \\ 0 & 0 & 1 \end{bmatrix}.$$

2. **Successive Link Frames (`frames` on $(S^1)^7$):**  
   The pose $T_{0,i}(X) \in SE(3)$ of link $i$ relative to the base is computed recursively without any matrix exponential or trigonometric function:
   $$T_{0,0} = I_4, \qquad T_{0,i}(X) = T_{0,i-1}(X) \, M_i \, T_z(x_i) \quad (i=1,\dots,7), \qquad T_{0,8}(X) = T_{0,7}(X) \, M_8.$$

3. **General Screw Exponential on $S^1$ (`MatrixExp6_S1` for Space PoE and Perturbed Screws):**  
   For a general screw $S_i = (\hat{\omega}_i, v_i) \in \mathbb{R}^6$ with $\|\hat{\omega}_i\|_2 = 1$ and $\hat{\omega}_i^T v_i = 0$ (pure revolute joint in space frame, since $S_i = [\hat{\omega}_i; -\hat{\omega}_i \times r_i]$), let $\Omega_i = [\hat{\omega}_i]_\times \in \mathfrak{so}(3)$. Because $\Omega_i^3 = -\Omega_i$ and $\Omega_i v_i$ is perpendicular to $\hat{\omega}_i$, $\Omega_i^2 v_i = -v_i$, which yields:
   $$G(x_i) v_i = \left(I_3 q_i + (1 - c_i)\Omega_i + (q_i - s_i)\Omega_i^2\right) v_i = q_i \underbrace{(I_3 + \Omega_i^2) v_i}_{=\, (\hat{\omega}_i \hat{\omega}_i^T) v_i \,=\, 0} + (1 - c_i)\Omega_i v_i - s_i \Omega_i^2 v_i = s_i v_i + (1 - c_i)\Omega_i v_i = \left(I_3 - R(\hat{\omega}_i, x_i)\right) (\Omega_i v_i).$$
   **Theorem 3.1 (Purely Algebraic Space Screw Exponential on $S^1$):**  
   *For any revolute screw $S_i = (\hat{\omega}_i, v_i)$ with $\|\hat{\omega}_i\|_2 = 1$ and zero pitch ($\hat{\omega}_i^T v_i = 0$), the $SE(3)$ exponential $\exp([S_i] q_i)$ depends **only** on $x_i = (c_i, s_i)^T \in S^1$, with zero explicit dependence on the angle $q_i$:*
   $$\boxed{\exp([S_i], x_i) = \begin{bmatrix} I_3 + s_i \Omega_i + (1 - c_i)\Omega_i^2 & s_i v_i + (1 - c_i)\Omega_i v_i \\ 0_{1\times 3} & 1 \end{bmatrix}, \qquad \Omega_i = [\hat{\omega}_i]_\times.}$$
   *What if a test artificially perturbs $S_i$ so that $\hat{\omega}_i^T v_i \ne 0$ (non-zero pitch helical screw, as in `validate_panda.m` line 65: `altered.Slist(6,2) = altered.Slist(6,2) + 0.01`)?*  
   Then the parallel component $(\hat{\omega}_i^T v_i)\hat{\omega}_i$ is non-zero and shifts along the screw axis by $q_i (\hat{\omega}_i^T v_i)\hat{\omega}_i$:
   $$\exp([S_i], x_i) = \begin{bmatrix} I_3 + s_i \Omega_i + (1 - c_i)\Omega_i^2 & s_i v_i + (1 - c_i)\Omega_i v_i + (\varpi_i(x_i) - s_i)(\hat{\omega}_i^T v_i)\hat{\omega}_i \\ 0_{1\times 3} & 1 \end{bmatrix},$$
   where $\varpi_i(x_i)$ is the lifted joint angle (Section 5.2). Notice that when $\hat{\omega}_i^T v_i = 0$ (all physical revolute joints), the $(\varpi_i(x_i) - s_i)$ term vanishes identically!

4. **Inter-Link Adjoint Transform in RNEA (`rnea_shared_kinematics`):**  
   In `rnea_shared_kinematics_MR.m`, line 91 evaluates:
   $$\operatorname{Ad}_{T_{i,i-1}(q_i)} = \operatorname{Ad}\!\left(\exp(-[A_i] q_i) \, M_i^{-1}\right).$$
   For the nominal Panda model where $A_i = e_3 = [0,0,1,0,0,0]^T$, $\exp(-[A_i] q_i) = T_z(\bar{x}_i) = \operatorname{diag}(R_z(x_i)^T, 1)$. Even more generally, for any zero-pitch body screw $A_i$, $\exp(-[A_i], x_i) = \exp([A_i], \bar{x}_i)$ is purely algebraic in $(c_i, s_i)$. And if $A_i$ has non-zero pitch due to a test perturbation of `Mlist` or `Slist`, the general formula above provides exact bitwise-compatible fallback.

5. **Space and Body Twist Jacobians on $(S^1)^7$:**  
   Since the space screw exponential $\exp([S_j], x_j)$ is purely algebraic in $x_j \in S^1$, the space Jacobian $J_s(X) \in \mathbb{R}^{6\times 7}$ and body Jacobian $J_b(X) \in \mathbb{R}^{6\times 7}$ are purely polynomial functions of $X \in (S^1)^7$:
   $$J_{s,1}(X) = S_1, \qquad J_{s,i}(X) = \operatorname{Ad}_{\exp([S_1],x_1)\cdots\exp([S_{i-1}],x_{i-1})} S_i \quad (i=2,\dots,7),$$
   $$J_b(X) = \operatorname{Ad}_{T(X)^{-1}} J_s(X), \qquad V_s = J_s(X)\omega, \qquad V_b = J_b(X)\omega.$$

---

## 4. Global Lagrangian Dynamics on $T(S^1)^7$ and Equivalence with Spatial RNEA

### 4.1 Variational Derivation on $T(S^1)^n$ (Lee et al., Chapter 4)

Let $X = (x_1, \dots, x_n) \in (S^1)^n$ with $n = 7$. An admissible infinitesimal variation of a curve $x_i(t) \in S^1$ with fixed endpoints is generated by the exponential map of $SO(2)$ (Lee Eq. 4.2, 4.10–4.11):
$$x_i^\epsilon(t) = \exp(\epsilon \gamma_i(t) J) x_i(t) \implies \delta x_i = \left.\frac{d}{d\epsilon}\right|_{\epsilon=0} x_i^\epsilon = \gamma_i J x_i \in T_{x_i} S^1, \qquad \delta \omega_i = \dot{\gamma}_i \in \mathbb{R},$$
for arbitrary smooth curves $\gamma_i: [t_0, t_f] \to \mathbb{R}$ vanishing at $t_0, t_f$.

For the modified Lagrangian $\tilde{L}(X, \omega) = K(X, \omega) - V(X) = \frac{1}{2} \omega^T M(X) \omega - V(X)$ subject to generalized non-conservative joint torques $\tau_i^{\text{nc}} = \tau_i - f_i(\omega_i) + (J_b(X)^T F_{\text{env}})_i$ performing virtual work $\delta W = \sum_{i=1}^n \gamma_i \tau_i^{\text{nc}}$, the Lagrange–d'Alembert principle $\delta \int_{t_0}^{t_f} \tilde{L}(X, \omega)\,dt + \int_{t_0}^{t_f} \delta W\,dt = 0$ yields:
$$\int_{t_0}^{t_f} \sum_{i=1}^n \left[ -\frac{d}{dt}\left(\frac{\partial \tilde{L}}{\partial \omega_i}\right) + (J x_i)^T \frac{\partial \tilde{L}}{\partial x_i} + \tau_i^{\text{nc}} \right] \gamma_i \, dt = 0.$$
Since $(J x_i)^T = -x_i^T J$, we obtain the forced **Global Euler–Lagrange Equations on $T(S^1)^n$** (Lee Proposition 4.2, Eq. 4.12):
$$\boxed{\frac{d}{dt}\left(\frac{\partial \tilde{L}(X,\omega)}{\partial \omega_i}\right) + x_i^T J \frac{\partial \tilde{L}(X,\omega)}{\partial x_i} = \tau_i - f_i(\omega_i) + \left(J_b(X)^T F_{\text{env}}\right)_i, \qquad i = 1,\dots,n.}$$

### 4.2 Theorem: Exact Equivalence of Reduced $S^1$ Dynamics and Spatial RNEA

**Theorem 4.1 (Dynamic Equivalence under the Covering Map):**  
*Let $\pi: \mathbb{R}^n \to (S^1)^n$ be the smooth covering map $\pi(q) = (\iota(q_1), \dots, \iota(q_n))$ where $\iota(q_i) = (\cos q_i, \sin q_i)^T$. For any smooth function $\Phi: (\mathbb{R}^2)^n \to \mathbb{R}$ whose restriction to $(S^1)^n$ represents a physical quantity (kinetic energy, mass matrix entry $M_{jk}(X)$, or gravitational potential $V(X)$), the chain rule yields:*
$$\frac{\partial (\Phi \circ \pi)}{\partial q_i}(q) = \left(\frac{\partial \Phi}{\partial x_i}(\pi(q))\right)^T \frac{d \iota(q_i)}{d q_i} = \left(\frac{\partial \Phi}{\partial x_i}(X)\right)^T (J x_i) = -x_i^T J \frac{\partial \Phi(X)}{\partial x_i}.$$
*Consequently, the directional derivative operator $\mathcal{D}_i \Phi(X) := (J x_i)^T \frac{\partial \Phi(X)}{\partial x_i} = -x_i^T J \frac{\partial \Phi(X)}{\partial x_i}$ on $(S^1)^n$ coincides identically with $\frac{\partial (\Phi \circ \pi)}{\partial q_i}$ at $X = \pi(q)$:*
1. **Gravity Vector:**  
   $$g_i(X) = -x_i^T J \frac{\partial V(X)}{\partial x_i} = (J x_i)^T \frac{\partial V(X)}{\partial x_i} = \frac{\partial V(\pi(q))}{\partial q_i}.$$
2. **Mass Matrix Time Derivative and Coriolis/Centrifugal Terms:**  
   $$\dot{M}_{ij}(X, \omega) = \sum_{k=1}^n \left(\frac{\partial M_{ij}(X)}{\partial x_k}\right)^T \dot{x}_k = \sum_{k=1}^n \left((J x_k)^T \frac{\partial M_{ij}(X)}{\partial x_k}\right) \omega_k = \sum_{k=1}^n (\mathcal{D}_k M_{ij}(X)) \omega_k,$$
   $$c_i(X, \omega) = \sum_{j=1}^n \dot{M}_{ij}(X,\omega)\omega_j - \frac{1}{2} \omega^T (\mathcal{D}_i M(X)) \omega = \sum_{j,k=1}^n \Gamma_{ijk}(X) \omega_j \omega_k,$$
   where $\Gamma_{ijk}(X) = \frac{1}{2}\left(\mathcal{D}_k M_{ij}(X) + \mathcal{D}_j M_{ik}(X) - \mathcal{D}_i M_{jk}(X)\right)$ are the Christoffel symbols evaluated globally on $(S^1)^n$.
3. **Spatial RNEA Invariance:**  
   In the spatial Newton–Euler recursion (`InverseDynamicsSharedKinematics_MR.m`), the forward pass propagates spatial twists $V_i \in \mathfrak{se}(3)$ and spatial accelerations $\dot{V}_i \in \mathfrak{se}(3)$:
   $$V_i = \operatorname{Ad}_{T_{i,i-1}(x_i)} V_{i-1} + A_i \omega_i, \qquad \dot{V}_i = \operatorname{Ad}_{T_{i,i-1}(x_i)} \dot{V}_{i-1} + A_i \dot{\omega}_i + \operatorname{ad}_{V_i}(A_i \omega_i),$$
   and the backward pass propagates spatial wrenches $F_i \in \mathfrak{se}(3)^*$:
   $$F_i = \operatorname{Ad}_{T_{i+1,i}(x_{i+1})}^T F_{i+1} + G_i \dot{V}_i - \operatorname{ad}_{V_i}^T (G_i V_i), \qquad \tau_i = A_i^T F_i.$$
   Because $V_i$, $\dot{V}_i$, and $F_i$ depend on the configuration **solely** through the inter-link Adjoint matrices $\operatorname{Ad}_{T_{i,i-1}(x_i)}$, the spatial RNEA recursion is already intrinsically defined on $(S^1)^7 \times \mathbb{R}^7$!

### 4.3 Structure of the Gravitational Potential $V(X)$

In terms of the link frames $T_{0,i}(X) = \begin{bmatrix} R_{0,i}(X) & p_{0,i}(X) \\ 0 & 1 \end{bmatrix} = M_1 T_z(x_1) \cdots M_i T_z(x_i)$, the center of mass of link $i$ in the base frame is $r_{\text{com},i}(X) = R_{0,i}(X) c_{\text{com},i} + p_{0,i}(X)$, and the potential energy is:
$$V(X) = -\sum_{i=1}^7 m_i g_{\text{grav}}^T r_{\text{com},i}(X).$$
Unlike a planar chain measured in absolute world angles where $V$ is a linear sum of $(c_i, s_i)$, on a 3D serial chain with relative joint coordinates $x_i \in S^1$, $r_{\text{com},i}(X)$ is **multilinear** in $(x_1, \dots, x_i)$: holding $x_{j \ne k}$ fixed, $V(X)$ is strictly affine-linear in $x_k = (c_k, s_k)^T$.

---

## 5. Rigorous Theory of Bounded Revolute Joints on $S^1$

### 5.1 Geometric Characterization of Admissible Arcs

For each joint $i \in \{1,\dots,7\}$, let $[q_{\min,i}, q_{\max,i}] \subset \mathbb{R}$ be the physical joint interval from `sources/parameters.json`. Define the midpoint $m_i$, half-span $h_i$, and center unit vector $u_i \in S^1$:
$$m_i = \frac{q_{\min,i} + q_{\max,i}}{2}, \qquad h_i = \frac{q_{\max,i} - q_{\min,i}}{2}, \qquad u_i = \begin{bmatrix} \cos m_i \\ \sin m_i \end{bmatrix} \in S^1.$$

| Joint $i$ | $q_{\min,i}$ [rad] | $q_{\max,i}$ [rad] | Center $m_i$ [rad] | Half-span $h_i$ [rad] | Total span $2h_i$ [rad] | $\cos h_i$ | $2h_i > \pi$? | Topological / Branch-Cut Feature |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|---|
| **1** | $-2.8973$ | $+2.8973$ | $0.0000$ | $2.8973$ | $5.7946$ | $-0.970309$ | **Yes** | Forbidden arc is only $0.4886\text{ rad}$ ($28.0^\circ$) wide around $\pm\pi$ |
| **2** | $-1.7628$ | $+1.7628$ | $0.0000$ | $1.7628$ | $3.5256$ | $-0.190826$ | **Yes** | Symmetric; span $> \pi$ |
| **3** | $-2.8973$ | $+2.8973$ | $0.0000$ | $2.8973$ | $5.7946$ | $-0.970309$ | **Yes** | Same as Joint 1 |
| **4** | $-3.0718$ | $-0.0698$ | $-1.5708$ | $1.5010$ | $3.0020$ | $+0.069740$ | No | Entirely negative ($q_{\max,4} < 0$); $\cos h_4 > 0$ |
| **5** | $-2.8973$ | $+2.8973$ | $0.0000$ | $2.8973$ | $5.7946$ | $-0.970309$ | **Yes** | Same as Joint 1 |
| **6** | $-0.0175$ | $+3.7525$ | $+1.8675$ | $1.8850$ | $3.7700$ | $-0.309059$ | **Yes** | **Crosses $+\pi$** ($q_{\max,6} = 3.7525 > \pi$); naive `atan2(s,c)` wraps to $-2.5307$! |
| **7** | $-2.8973$ | $+2.8973$ | $0.0000$ | $2.8973$ | $5.7946$ | $-0.970309$ | **Yes** | Same as Joint 1 |

**Proposition 5.1 (Exact Algebraic Arc Characterization):**  
*Because $0 < h_i < \pi$ for all $i \in \{1,\dots,7\}$, the map $\iota: [q_{\min,i}, q_{\max,i}] \to S^1$, $q_i \mapsto (\cos q_i, \sin q_i)^T$, is injective, and its image is the closed circular arc $\mathcal{A}_i \subset S^1$ uniquely characterized by:*
$$\boxed{\mathcal{A}_i = \left\{ x_i \in \mathbb{R}^2 : x_i^T x_i = 1, \;\; u_i^T x_i \ge \cos h_i \right\}.}$$
*Proof:* For $x_i = (\cos q_i, \sin q_i)^T$ with $q_i = m_i + \delta_i$, $\delta_i \in (-\pi, \pi]$, we have $u_i^T x_i = \cos(q_i - m_i) = \cos \delta_i$. Since $\cos$ is strictly decreasing on $[0, \pi]$ and even, $\cos \delta_i \ge \cos h_i$ with $h_i \in (0, \pi)$ holds if and only if $|\delta_i| \le h_i$, i.e., $q_i \in [m_i - h_i, m_i + h_i] = [q_{\min,i}, q_{\max,i}]$. $\blacksquare$

### 5.2 Branch-Cut-Safe Lifting Map $\varpi_i: \mathcal{A}_i \to [q_{\min,i}, q_{\max,i}]$

To recover the physical joint angle $q_i \in [q_{\min,i}, q_{\max,i}]$ from $x_i = (c_i, s_i)^T \in \mathcal{A}_i$ without ambiguity or branch-cut discontinuity (especially on Joint 6 where $q_6$ crosses $\pi$), we rotate $x_i$ into the reference frame of the arc midpoint $u_i = (\cos m_i, \sin m_i)^T$:
$$\tilde{x}_i = \Phi(u_i)^T x_i = \begin{bmatrix} u_i^T x_i \\ (J u_i)^T x_i \end{bmatrix} = \begin{bmatrix} u_i^T x_i \\ u_i^T J^T x_i \end{bmatrix} = \begin{bmatrix} c_i \cos m_i + s_i \sin m_i \\ s_i \cos m_i - c_i \sin m_i \end{bmatrix} = \begin{bmatrix} \cos(q_i - m_i) \\ \sin(q_i - m_i) \end{bmatrix}.$$
Since $q_i \in [q_{\min,i}, q_{\max,i}] \iff \delta_i := q_i - m_i \in [-h_i, h_i] \subset (-\pi, \pi)$, the principal branch of $\operatorname{atan2}$ recovers $\delta_i$ smoothly and uniquely everywhere on an open neighborhood of $\mathcal{A}_i$:
$$\boxed{q_i = \varpi_i(x_i) := m_i + \operatorname{atan2}\!\left(u_i^T J^T x_i, \; u_i^T x_i\right) = m_i + \operatorname{atan2}\!\left(s_i \cos m_i - c_i \sin m_i, \; c_i \cos m_i + s_i \sin m_i\right).}$$

- **Safety Margin to the Branch Cut:** The discontinuity of $\varpi_i$ occurs at $\tilde{x}_i = (-1, 0)^T \iff q_i = m_i \pm \pi$, which is the **midpoint of the forbidden sector** $S^1 \setminus \mathcal{A}_i$. Its angular distance to the nearest admissible boundary $q_{\min,i}$ or $q_{\max,i}$ is $\pi - h_i > 0$ ($\ge 0.2443\text{ rad} \approx 14.0^\circ$ for Joints 1, 3, 5, 7, and $1.2566\text{ rad} \approx 72.0^\circ$ for Joint 6).
- **Direct Limit Margin Metric on $S^1$:** Without even adding $m_i$, the signed angular offset from the center is $\delta_i(x_i) = \operatorname{atan2}(u_i^T J^T x_i, u_i^T x_i)$, and the exact radian margin to the nearest joint limit is:
  $$\boxed{\mu_i(x_i) := h_i - \left|\operatorname{atan2}\!\left(u_i^T J^T x_i, \; u_i^T x_i\right)\right| = \min(q_i - q_{\min,i}, \; q_{\max,i} - q_i),}$$
  with $\mu_i(x_i) \ge 0 \iff u_i^T x_i \ge \cos h_i$.

### 5.3 Geodesic Logarithm vs. Admissible-Arc Difference (Avoiding Forbidden-Stop Traversal)

Suppose $x_{1,i}, x_{2,i} \in \mathcal{A}_i$ are two admissible states of joint $i$ (for example, current state $x_i$ and target/reference state $x_{\text{ref},i}$). There are **two** distinct mathematical operations for their angular difference:

1. **Intrinsic $SO(2)$ Geodesic Logarithm (`geodesic_diff`):**  
   $$\Delta \theta_{\text{geo},i}(x_{2,i}, x_{1,i}) = \operatorname{atan2}\!\left(x_{1,i}^T J^T x_{2,i}, \; x_{1,i}^T x_{2,i}\right) \in (-\pi, \pi].$$
   And the smooth chordal error of Lee (Chapter 4):
   $$e_{\text{chord},i}(x_{2,i}, x_{1,i}) = x_{1,i}^T J^T x_{2,i} = \sin(q_{2,i} - q_{1,i}).$$
   **Topological Pathology on Bounded Joints with $2h_i > \pi$:**  
   When $|q_{2,i} - q_{1,i}| > \pi$ (which is possible on Joints 1, 2, 3, 5, 6, 7), the shortest geodesic on $S^1$ between $x_{1,i}$ and $x_{2,i}$ has length $2\pi - |q_{2,i} - q_{1,i}| < \pi$ and **traverses the forbidden arc $S^1 \setminus \mathcal{A}_i$**! In that regime:
   - $\Delta \theta_{\text{geo},i}$ has the **opposite sign** to the only physically feasible motion $q_{2,i} - q_{1,i}$;
   - $e_{\text{chord},i} = \sin(q_{2,i} - q_{1,i})$ **changes sign** at $|q_{2,i} - q_{1,i}| = \pi$ and creates a spurious unstable equilibrium at antipodal configurations, pushing the joint into the mechanical hard stop!

2. **Admissible-Arc Difference (`box_diff` / Homotopy-Consistent Difference on $\mathcal{A}_i$):**  
   Because $\mathcal{A}_i \subsetneq S^1$ is contractible (diffeomorphic to the compact interval $[q_{\min,i}, q_{\max,i}]$ via $\varpi_i$), the unique displacement along the admissible physical arc $\mathcal{A}_i$ is:
   $$\boxed{\Delta q_{\mathcal{A},i}(x_{2,i}, x_{1,i}) := \varpi_i(x_{2,i}) - \varpi_i(x_{1,i}) = \delta_i(x_{2,i}) - \delta_i(x_{1,i}) \in [-2h_i, 2h_i].}$$
   - **Theorem 5.2 (Agreement on Local Neighborhoods, Safety on Large Motions):**  
     *Whenever $|\Delta q_{\mathcal{A},i}| < \pi$, $\Delta q_{\mathcal{A},i}(x_{2,i}, x_{1,i}) = \Delta \theta_{\text{geo},i}(x_{2,i}, x_{1,i})$ identically. Whenever $|\Delta q_{\mathcal{A},i}| \ge \pi$, $\Delta \theta_{\text{geo},i}$ points across the forbidden stop $S^1 \setminus \mathcal{A}_i$, whereas $\Delta q_{\mathcal{A},i}$ points along the sole physically admissible path in $\mathcal{A}_i$.*
   - **Forbidden-Arc Detection Predicate:**  
     A geodesic path from $x_{1,i} \in \mathcal{A}_i$ to $x_{2,i} \in \mathcal{A}_i$ crosses the forbidden sector $S^1 \setminus \mathcal{A}_i$ if and only if:
     $$\left|\delta_i(x_{2,i}) - \delta_i(x_{1,i})\right| > \pi.$$

### 5.4 Geometric Admissibility vs. Dynamic Safety

We explicitly distinguish three levels of constraints (preventing overclaiming in documentation or tests):
1. **Manifold Constraint (Holonomic Geometry):** $x_i^T x_i = 1$. Enforced by the Lie-group representation and geometric integrator to machine precision ($\sim 10^{-16}$).
2. **Static Configuration Admissibility:** $x_i \in \mathcal{A}_i \iff u_i^T x_i \ge \cos h_i$, alongside velocity bounds $|\omega_i| \le \omega_{\max,i}$ and torque bounds $|\tau_i| \le \tau_{\max,i}$. Monitored at every integration stage and sample step, matching Version A (`panda.simulate`).
3. **Dynamic Safety / Control Invariance (Viability):** Satisfaction of $u_i^T x_i(t) \ge \cos h_i$ at time $t$ with $\omega_i(t)$ pointing toward the boundary does **not** guarantee future invariance if the available braking torque $\tau_{\max,i}$ cannot arrest the momentum before reaching the stop (roughly requiring $\mu_i(x_i) \ge \omega_i^2 / (2 \ddot{q}_{\max,\text{brake},i})$). Just as in Version A, the simulator monitors stage margins and enforces torque saturation, without claiming a hard-contact collision model or control-barrier invariance certificate.

---

## 6. Geometric Integration on $(S^1)^7 \times \mathbb{R}^7$

### 6.1 Failure of Ambient RK4 on $\mathbb{R}^{14}$

If one integrates $\dot{x}_i = \omega_i J x_i$ in ambient Euclidean space $\mathbb{R}^2$ using classical 4-stage Runge–Kutta (RK4) with constant $\omega_i$, a single step of size $h$ multiplies $x_i$ by $1 - \frac{(\omega_i h)^2}{2} + \frac{(\omega_i h)^4}{24} + J\left(\omega_i h - \frac{(\omega_i h)^3}{6}\right)$, whose squared norm is:
$$\|x_{i,k+1}\|_2^2 = 1 - \frac{(\omega_i h)^6}{72} + O((\omega_i h)^8).$$
Thus, ambient RK4 violates $c_i^2 + s_i^2 = 1$ at $O(h^5)$ per step, causing $R_z(x_i)$ to lose orthogonality ($\det R_z(x_i) = c_i^2 + s_i^2 \ne 1$) and corrupting spatial transformations and energy balances.

### 6.2 Abelian Munthe–Kaas Lie-Group RK4 (RK4-MK) on $(S^1)^7 \times \mathbb{R}^7$

The state space is the left-trivialized tangent bundle $\mathsf{M} = (S^1)^7 \times \mathbb{R}^7$ (augmented by the 3 scalar work/dissipation integrals $W_{\text{joint}}, E_{\text{fric}}, W_{\text{env}} \in \mathbb{R}^3$).
Because $(SO(2))^7$ is an **abelian Lie group**, its Lie algebra $\mathfrak{g} \cong \mathbb{R}^7$ has vanishing Lie brackets ($[\Omega_a, \Omega_b] = 0$), so the inverse differential of the exponential map is the identity operator everywhere:
$$\mathrm{dexp}_{-\Delta\theta}^{-1}(\omega) = \omega.$$

**Algorithm 6.1 (4-Stage Munthe–Kaas Lie-Group RK4 Step on $(S^1)^7 \times \mathbb{R}^7$):**  
Given state $(X_k, \omega_k, w_k) \in (S^1)^7 \times \mathbb{R}^7 \times \mathbb{R}^3$ at time $t_j$, step size $h$, and held command $\tau_{\text{cmd}} \in \mathbb{R}^7$:
1. Define the exponential group action $\operatorname{Exp}_{S^1}: \mathbb{R}^7 \times (S^1)^7 \to (S^1)^7$:
   $$X(\Delta\theta) = \operatorname{Exp}_{S^1}(\Delta\theta, X_k), \qquad x_i(\Delta\theta_i) = \begin{bmatrix} \cos\Delta\theta_i & -\sin\Delta\theta_i \\ \sin\Delta\theta_i & \cos\Delta\theta_i \end{bmatrix} x_{i,k}, \quad i=1,\dots,7.$$
2. Evaluate the 4 Munthe–Kaas RK4 stages in the local tangent chart $(\Delta\theta, \omega, w) \in \mathbb{R}^7 \times \mathbb{R}^7 \times \mathbb{R}^3$ rooted at $\Delta\theta^{(1)} = 0_{7\times 1}$ ($X^{(1)} = X_k$):
   - **Stage 1:** $X^{(1)} = X_k$, $\omega^{(1)} = \omega_k$,  
     $k_{1,\theta} = \omega^{(1)}$, $\quad k_{1,\omega} = f_{\text{dyn}}(t_j, X^{(1)}, \omega^{(1)}, \tau_{\text{cmd}})$, $\quad k_{1,w} = f_{\text{pow}}(t_j, X^{(1)}, \omega^{(1)}, \tau_{\text{cmd}})$.
   - **Stage 2:** $\Delta\theta^{(2)} = \frac{h}{2} k_{1,\theta}$, $\quad X^{(2)} = \operatorname{Exp}_{S^1}(\Delta\theta^{(2)}, X_k)$, $\quad \omega^{(2)} = \omega_k + \frac{h}{2} k_{1,\omega}$,  
     $k_{2,\theta} = \omega^{(2)}$, $\quad k_{2,\omega} = f_{\text{dyn}}(t_j + \tfrac{h}{2}, X^{(2)}, \omega^{(2)}, \tau_{\text{cmd}})$, $\quad k_{2,w} = f_{\text{pow}}(t_j + \tfrac{h}{2}, X^{(2)}, \omega^{(2)}, \tau_{\text{cmd}})$.
   - **Stage 3:** $\Delta\theta^{(3)} = \frac{h}{2} k_{2,\theta}$, $\quad X^{(3)} = \operatorname{Exp}_{S^1}(\Delta\theta^{(3)}, X_k)$, $\quad \omega^{(3)} = \omega_k + \frac{h}{2} k_{2,\omega}$,  
     $k_{3,\theta} = \omega^{(3)}$, $\quad k_{3,\omega} = f_{\text{dyn}}(t_j + \tfrac{h}{2}, X^{(3)}, \omega^{(3)}, \tau_{\text{cmd}})$, $\quad k_{3,w} = f_{\text{pow}}(t_j + \tfrac{h}{2}, X^{(3)}, \omega^{(3)}, \tau_{\text{cmd}})$.
   - **Stage 4:** $\Delta\theta^{(4)} = h k_{3,\theta}$, $\quad X^{(4)} = \operatorname{Exp}_{S^1}(\Delta\theta^{(4)}, X_k)$, $\quad \omega^{(4)} = \omega_k + h k_{3,\omega}$,  
     $k_{4,\theta} = \omega^{(4)}$, $\quad k_{4,\omega} = f_{\text{dyn}}(t_j + h, X^{(4)}, \omega^{(4)}, \tau_{\text{cmd}})$, $\quad k_{4,w} = f_{\text{pow}}(t_j + h, X^{(4)}, \omega^{(4)}, \tau_{\text{cmd}})$.
3. **Manifold Update:**
   $$\Delta\theta_{\text{step}} = \frac{h}{6}\left(k_{1,\theta} + 2 k_{2,\theta} + 2 k_{3,\theta} + k_{4,\theta}\right),$$
   $$\tilde{X}_{k+1} = \operatorname{Exp}_{S^1}(\Delta\theta_{\text{step}}, X_k), \qquad x_{i,k+1} = \frac{\tilde{x}_{i,k+1}}{\|\tilde{x}_{i,k+1}\|_2},$$
   $$\omega_{k+1} = \omega_k + \frac{h}{6}\left(k_{1,\omega} + 2 k_{2,\omega} + 2 k_{3,\omega} + k_{4,\omega}\right), \qquad w_{k+1} = w_k + \frac{h}{6}\left(k_{1,w} + 2 k_{2,w} + 2 k_{3,w} + k_{4,w}\right).$$

### 6.3 Key Properties of Algorithm 6.1

1. **Fourth-Order Accuracy & Exact Equivalence with Lifted RK4:**  
   In exact arithmetic, $\operatorname{Exp}_{S^1}(\Delta\theta, \pi(q_k)) = \pi(q_k + \Delta\theta)$. Therefore, Algorithm 6.1 is **mathematically isomorphic** to classical RK4 on the universal cover $(\mathbb{R}^7, \mathbb{R}^7)$ while operating purely on $(S^1)^7 \times \mathbb{R}^7$ without storing $q \in \mathbb{R}^7$ as the state!
2. **Exact Unit-Circle Preservation Without Phase Distortion:**  
   In IEEE-754 double precision, multiplying $x_{i,k}$ by $\begin{bmatrix} \cos\Delta\theta_i & -\sin\Delta\theta_i \\ \sin\Delta\theta_i & \cos\Delta\theta_i \end{bmatrix}$ preserves $\|\tilde{x}_{i,k+1}\|_2 = 1$ to within $1\,\text{ulp}$ ($\sim 2.2 \times 10^{-16}$) per step. Radial renormalization $x_{i,k+1} = \tilde{x}_{i,k+1} / \|\tilde{x}_{i,k+1}\|_2$ scales both components by the same positive scalar $1 + O(\epsilon_{\text{mach}})$, so it **never alters the phase angle** $\operatorname{atan2}(s_i, c_i)$ while bounding $|c_i^2 + s_i^2 - 1| \le 2\epsilon_{\text{mach}}$ over arbitrarily long simulations ($N \to \infty$).
3. **Stage-Level Limit Monitoring on $(S^1)^7$:**  
   At all 5 stage configurations $\{X^{(1)}, X^{(2)}, X^{(3)}, X^{(4)}, X_{k+1}\}$, the exact radian margin $\mu_i(x_i^{(r)}) = h_i - |\delta_i(x_i^{(r)})|$ is evaluated directly on $(S^1)^7$, preserving 100% parity with the stage diagnostics of `panda.simulate`.

---

## 7. Software Architecture, Interfaces, and Contracts (Specifications for Hitos 2–5)

To keep Version A (`+panda`) 100% untouched and avoid duplicating the RNEA velocity/wrench loops (`InverseDynamicsSharedKinematics_MR.m` and `MassMatrixZeroStateSharedKinematics_MR.m`), we establish the following package and kernel architecture:

### 7.1 Package and Kernel Layout

1. **Original Package (`+panda/`):**  
   Remains completely unaltered in behavior and public API (**Version A**).
2. **Global $S^1$ Package (`+panda_s1/`):**  
   Implements **Version B (Global $(S^1)^7$ Geometric Dynamics)** with explicit $S^1$ configuration state $X \in \mathbb{R}^{2\times 7}$ (also accepting vectorized $14\times 1$ via `panda_s1.config_matrix(X)`):
   - `+panda_s1/model.m`: Wraps `panda.model(options)` and precomputes the $S^1$ limit geometry fields:
     - `m.qmid` ($7\times 1$, $m_i$), `m.qhalf` ($7\times 1$, $h_i$), `m.umid` ($2\times 7$, $u_i = [\cos m_i; \sin m_i]$), `m.cos_half` ($7\times 1$, $\cos h_i$), `m.is_pure_z` ($1\times 7$ logical flag verifying $A_i = e_3$).
   - `+panda_s1/from_q.m`: Lifting embedding $X = \text{panda\_s1.from\_q}(q) = [\cos q^T; \sin q^T] \in \mathbb{R}^{2\times 7}$.
   - `+panda_s1/to_q.m`: Branch-cut-safe centered chart map $q = \text{panda\_s1.to\_q}(X, m) \in \mathbb{R}^{7\times 1}$ using Eq. (5.2).
   - `+panda_s1/config_matrix.m`: Validates that $X$ has size $2\times 7$ (or reshapes $14\times 1 \to 2\times 7$), checks finiteness, and optionally returns unit-circle residual $\max_i |x_i^T x_i - 1|$.
   - `+panda_s1/check_limits.m`: Evaluates algebraic arc admissibility $u_i^T x_i \ge \cos h_i - \text{tol}$, unit-norm error $|x_i^T x_i - 1|$, centered offsets $\delta_i(x_i)$, and radian margins $\mu_i(x_i) = h_i - |\delta_i(x_i)|$.
   - `+panda_s1/geodesic_diff.m`: Intrinsic $SO(2)$ logarithm $\Delta\theta_{\text{geo}}(X_2, X_1) \in (-\pi, \pi]^7$ and boolean mask `crosses_forbidden_arc`.
   - `+panda_s1/box_diff.m`: Admissible-arc displacement $\Delta q_{\mathcal{A}}(X_2, X_1, m) = \varpi(X_2) - \varpi(X_1) \in [-2h, 2h]$.
   - `+panda_s1/exp_s1.m`: Group action $\operatorname{Exp}_{S^1}(\Delta\theta, X)$ on $(S^1)^7$ with optional radial normalization.
   - `+panda_s1/fk.m`: Forward kinematics $T = \text{panda\_s1.fk}(X, m)$ on $(S^1)^7$.
   - `+panda_s1/frames.m`: Link frame chain $[chain, T] = \text{panda\_s1.frames}(X, m)$ using direct algebraic $T_z(x_i)$.
   - `+panda_s1/jacobian.m`: Space and body twist Jacobians $J = \text{panda\_s1.jacobian}(X, m, frame)$ on $(S^1)^7$.
   - `+panda_s1/dynamics.m`: $[M, b, parts] = \text{panda\_s1.dynamics}(X, \omega, m)$ calling `rnea_shared_kinematics_S1` and reusing `MassMatrixZeroStateSharedKinematics_MR` and `InverseDynamicsSharedKinematics_MR` without code duplication!
   - `+panda_s1/inverse_dynamics.m`: $\tau = \text{panda\_s1.inverse\_dynamics}(X, \omega, \dot{\omega}, m, F_{\text{tip}})$.
   - `+panda_s1/forward_dynamics.m`: $\dot{\omega} = \text{panda\_s1.forward\_dynamics}(X, \omega, \tau, m, F_{\text{env}})$.
   - `+panda_s1/energy.m`: $[H, K, V] = \text{panda\_s1.energy}(X, \omega, m)$.
   - `+panda_s1/simulate.m`: Munthe–Kaas Lie-group RK4 simulator on $(S^1)^7 \times \mathbb{R}^7$ tracking reference trajectories, recording `log.X` ($2\times 7\times N$), lifted `log.q` ($7\times N$), unit-norm residuals `log.max_unit_norm_error`, and all energy/limit metrics.
3. **Shared Kinematics Kernel on $S^1$ (`third_party/shape_kernels/rnea_shared_kinematics_S1.m`):**  
   - Accepts $X \in \mathbb{R}^{2\times 7}$, `Mlist`, `Slist`, and optional `mode`.
   - Performs the same safe invariant hoisting as `rnea_shared_kinematics_MR.m`, plus detects per joint whether `cached_Ai(:,i)` is pure $z$-rotation ($[0;0;1;0;0;0]$ within $10^{-14}$).
   - For pure $z$-rotation joints (all 7 nominal Panda joints), constructs $\operatorname{Ad}_{T_{i,i-1}(x_i)}$ directly from $x_i = (c_i, s_i)^T$ and `cached_invM(:,:,i)` without calling `MatrixExp6` or `VecTose3`!
   - If a test alters `Mlist` or `Slist` so that `cached_Ai(:,i)` is not pure $z$, automatically falls back to the general screw exponential using lifted angle $\varpi_i(x_i)$, ensuring 100% pass rate on geometry/screw-perturbation tests.
   - Implements bitwise state caching via `typecast(X(:), 'uint64')` ($14 \times 1$ `uint64` vector).
4. **Variant B-alg for Milestone 5 Benchmark (`rnea_shared_kinematics_rot_MR.m` or benchmark wrapper):**  
   - Accepts $q \in \mathbb{R}^7$, computes $X = [\cos q^T; \sin q^T]$, and invokes the specialized algebraic rotation kernel. This cleanly separates the speedup due to replacing generic `MatrixExp6` by specialized $R_z(c,s)$ (**Variant B-alg**) from the representational properties of evolving directly on $(S^1)^7$ (**Variant C / Version B**).

### 7.2 Mandatory Verification Properties and Test Contracts

| Test Category | Mathematical Property to Verify | Target Tolerance |
|---|---|:---:|
| **1. Manifold & Chart Consistency** | $\|\text{from\_q}(q)_i\|_2 = 1$; $\text{to\_q}(\text{from\_q}(q), m) = q$ across interior and boundary samples of all 7 joints (including Joint 4 $<0$ and Joint 6 $>\pi$). | $\le 10^{-15}\text{ rad}$ |
| **2. Arc Inequality Discrimination** | $u_i^T x_i \ge \cos h_i$ accepts all $q_i \in [q_{\min,i}, q_{\max,i}]$ and rejects $q_{\min,i} - \epsilon$, $q_{\max,i} + \epsilon$ ($\epsilon = 10^{-4}\text{ rad}$). | Exact classification |
| **3. Geodesic vs. Box Difference** | For pairs with $|q_2 - q_1| > \pi$ on Joints 1, 2, 3, 5, 6, 7: verify `geodesic_diff` flags `crosses_forbidden_arc = true` and has opposite sign to `box_diff`, while `box_diff` equals $q_2 - q_1$ to machine precision. | $\le 10^{-15}\text{ rad}$ |
| **4. Kinematic & Dynamic Parity** | Over 40+ random states: `fk`, `frames`, `jacobian` ('space' & 'body'), `dynamics` ($M, b, g, c$), `inverse_dynamics`, `forward_dynamics`, `energy` ($H, K, V$) between `+panda` at $q$ and `+panda_s1` at $X = \text{from\_q}(q)$. | $\le 10^{-14}$ (relative / Frobenius) |
| **5. Cache Invalidation Parity** | Perturbing `Mlist` and `Slist` (including non-zero pitch screw perturbation `Slist(6,2) += 0.01`) invalidates cache and matches `MassMatrix` oracle. | $\le 10^{-12}$ |
| **6. Lie-Group Integrator Invariance** | Across full 3.6 s trajectory and stressed trajectories: $\max_{i,k} |x_{i,k}^T x_{i,k} - 1| \le 10^{-15}$, trajectory parity with Version A, and energy balance residual $< 10^{-6}\text{ J}$. | $\le 10^{-15}$ (norm), $< 10^{-6}\text{ J}$ (energy) |
