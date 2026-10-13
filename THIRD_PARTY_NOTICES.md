# Source attribution and licenses

## Own code and reused project kernels

The new `+panda` package, examples, tests and utilities are MIT, copyright 2026 Johnny Garcia. Own simulator functions are adapted from the author's Kinova-Gen3-Geometric-Simulator release 0.1.0. The SHAPE spatial kernels and Flatness-MPC phase functions are reused with the author's authorization under this repository's selected MIT license. Their original content is preserved byte-for-byte and hashed in `sources/manifest.json`.

## Modern Robotics

`third_party/modern_robotics` preserves the author's existing local MATLAB copies used by SHAPE. Original comments and the NxRLab MIT license remain included. The local `InverseDynamics` scientific copy differs from upstream; this repository does not claim all bundled files are unmodified current upstream programs. Source: [NxRLab/ModernRobotics](https://github.com/NxRLab/ModernRobotics); copyright 2019 NxRLab. See the included `third_party/modern_robotics/LICENSE`.

## Franka configuration and geometry

Manufacturer Xacro, YAML and original COLLADA assets come from [frankarobotics/franka_ros](https://github.com/frankarobotics/franka_ros/tree/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98) at the pinned revision. They retain Apache-2.0, including the full notice in `sources/franka_ros_LICENSE`; they are not relicensed as MIT. Converted binary STL files are derived from those COLLADA files, with scene transforms applied and materials removed. Exact URLs, hashes and conversion records are in the manifests. The manufacturer Xacro files are source evidence, not a complete standalone ROS description package.

The Panda name identifies the simulated platform and does not imply manufacturer endorsement. Visual meshes do not implement collision response.

## Parameter literature

Numerical facts and equations are attributed to Gaz, Cognetti, Oliva, Robuffo Giordano and De Luca (2019), DOI 10.1109/LRA.2019.2931248, and the revised supplement/erratum. Their published feasible physical parameter set is implemented using independently authored geometric dynamics. The original GPL-generated dynamics/identification programs are neither bundled nor called. The original article and supplement PDFs are not redistributed or relicensed. See `docs/MODEL.md` and `docs/SOURCE_REVIEW.md` for the precise parameter-set choice.

No MATLAB distribution, toolboxes or commercial binaries are included.
