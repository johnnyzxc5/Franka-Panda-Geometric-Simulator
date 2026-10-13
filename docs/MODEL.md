# Geometric mechanical model

## Parameter choice

The model uses Gaz et al. (2019), revised supplement dated October 15, 2019: physical masses/COMs from Table VIII, COM inertias from Table IX, and sigmoid friction from Table I with the published joint-2 correction. The seven masses, COM vectors and inertia tensors are transcribed in `sources/parameters.json`. These are feasible identified equivalents, not a uniquely recovered manufacturer CAD mass distribution.

The supplement's page 1 distinguishes identified base coefficients from the coefficients obtained by substituting the feasible physical parameters. They need not be identical. For example, our physical-table `M(7,7)` is `0.004909651967360946 kg m²`, from `I7zz + m7*(c7x^2+c7y^2)`. The upstream generated matrix instead gives `0.001272243011263065`. That disagreement is not eliminated by changing coordinate frames. All dynamics here consistently use the physical-table set. The generated programs were examined separately; they are not runtime dependencies.

Table VIII marks `c1z` as unidentifiable and irrelevant to joint dynamics. We choose zero. Changing it on this fixed vertical first axis shifts the potential reference but not its gradient. Franka's bundled `inertial.yaml` uses another choice; it is retained as a source cross-check, not loaded as a second competing model.

## Frames and Lie algebra

The base is fixed. `T_ab` maps coordinates from frame b to frame a. Twists are `[omega; v]`, wrenches `[moment; force]`; adjoint and dual transformations preserve their power pairing. The manufacturer joint-origin facts are independently extracted from pinned Xacro into `sources/joint_origins.json`. The relative transforms agree with the modified-DH construction `Rx(alpha_i)*Tx(a_i)*Tz(d_i)` at zero joint angle.

Home poses are accumulated separately from relative transforms. A local joint axis becomes `w = R_home*[0;0;1]` and the space screw is `[w; -cross(w,p_home)]`. Flange motion is

```text
T(q) = exp([S1]q1) ... exp([S7]q7) T_home.
```

Body screws are `Ad(inv(T_home))*Slist`; `J_body = Ad(inv(T(q)))*J_space`. A seven-axis arm has a six-by-seven twist Jacobian. `Mlist(:,:,i)` is the home transform relative to the previous body frame, not relative to the base. The final fixed transform defines the bare flange `panda_link8`. Mesh scene-node transforms are applied during COLLADA-to-STL conversion before posing each link.

## Spatial inertia and energy

For a link-local COM offset c and `C=skew(c)`, the full spatial inertia about the link origin is

```text
G = [I_com - m*C*C,  m*C;
                 -m*C, m*eye(3)].
```

The cross blocks and parallel-axis shift are retained. Published COM inertias are used in their documented link-aligned coordinates. All tensors are positive definite and obey principal-moment triangle inequalities.

```text
M(q)*qdd + c(q,qd) + g(q) + f(qd) = tau + J_body(q)'*F_env
H = 0.5*qd'*M(q)*qd - sum(m_i*gravity'*p_com_i).
```

The default is a rigid link-side model, following the paper's Sec. III-A. It adds no independently assumed rotor or gearbox term. The optional constant diagonal `additional_apparent_inertia` is a user-selected extension, included in both mass and kinetic energy once. It does not reconstruct joint elasticity or an electrical actuator model. No efficiency constant scales the mass matrix.

## Friction

The paper's eq. (25) is evaluated in an equivalent stable form:

```text
f = 0.5*phi1 .* (tanh(0.5*phi2.*(qd+phi3)) - tanh(0.5*phi2.*phi3)).
```

Joint-2 `phi1=0.87224 N m` follows the authors' erratum. Since `phi1,phi2>0`, each component is monotone, vanishes at zero velocity and dissipates power. The law is smooth and saturating; it has no set-valued static-friction threshold. Coefficients describe the paper's link-side torque model, not total electrical losses or uncompensated motor friction.

## Control and integration

The example uses seven joint positions as flat outputs and the reused quintic phase generator. Computed torque maps the reference plus PD acceleration correction through inverse dynamics. It demonstrates the model interface; it does not implement MPC. Saturation occurs before multiplicative actuator effectiveness. Commands are held for 4 ms while RK4 integrates the plant in 1 ms stages by default.

The environmental wrench is expressed at the flange in its body frame and adds `J_body'*F_env`. Modern Robotics inverse dynamics instead expects the robot-to-environment wrench `Ftip=-F_env`; tests check the sign convention.

Work and dissipated energy use the same RK4 stage states as the plant:

```text
H(t)-H(0) - integral(tau_applied'*qd)
           - integral(F_env'*J_body*qd) + integral(f(qd)'*qd).
```

Effectiveness and environmental callbacks are evaluated at each stage. Joint positions and velocities are monitored at those stages, with no hard stops or invariant-set claim. Step sizes require rechecking after model, forcing or gain changes. The default demonstration's tracking performance assumes a matching controller and plant.

## Scope of physical fidelity

The model is suitable for rigid mechanical simulation with documented identified parameters. It is not a new identification or hardware validation. No separately added hand, payload, flexibility, contact, collision, electrical dynamics or sensor model is present. The [official FCI overview](https://frankarobotics.github.io/docs/overview.html) describes compensated joint-torque commands and link-side measurements; direct deployment needs an interface-specific conversion and its own validation.

Sources: [Gaz article](https://doi.org/10.1109/LRA.2019.2931248), [revised supplement and erratum](https://diag.uniroma1.it/gaz/panda2019.html), [pinned manufacturer source](https://github.com/frankarobotics/franka_ros/tree/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description).
