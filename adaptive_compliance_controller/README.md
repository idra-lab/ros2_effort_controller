# Adaptive Compliance Controller — torque output

`adaptive_compliance_controller/AdaptiveComplianceController` adapts
[Beber et al., ICRA 2024](https://doi.org/10.1109/ICRA57147.2024.10610167) and the
[original adaptive compliance code](https://github.com/lucabeber/cartesian_controllers/tree/036c1b6ca4ebc158802cf7968fbc767b3e8d8d78/cartesian_adaptive_compliance_controller)
to robots commanded in joint effort. Its task wrench is mapped through `J^T`;
there is no simulated joint position/velocity command loop.

The published experiments use a position-controlled UR3e with FDCC. KUKA and
Franka hardware experience mentioned in the repository README concerns the
effort-controller family. The new adaptive torque port has synthetic validation.

This is a separate plugin. It reuses the force controller's FT load compensation,
frame transforms, lifecycle, watchdogs and hardware effort limits. It replaces
that controller's task law with a three-variable QP and an energy tank.

## Controller and model

The translational wrench is `F = diag(k) * e - D * v`, with `e = x_desired - x`.
The QP minimizes force tracking error and distance from minimum stiffness,
subject to stiffness bounds, predicted-force bounds, tank availability and an
extraction-power limit (paper Eq. 9). The rotational spring remains fixed.
All stiffness/damping axes are **fixed robot-base axes**, so the QP and power
calculation use the same frame. FT messages still use the actual sensor frame.

The base frame's Z axis points upward. Tissue geometry is a height field
`z = surface(x,y)`; penetration is `surface_z - tcp_z`, and compressive target
force is negative along base Z. The unilateral Hunt–Crossley model is:

```
F_tissue = max(0, (kappa + viscosity * penetration_rate) * penetration^beta)
```

It is zero outside contact. Contact gating uses the compensated measured
reaction along positive base Z and nonnegative model penetration.

| `adaptive.mode` | Behavior in contact |
| --- | --- |
| `constant_force` (VS-CF) | Tracks the commanded linear force; the Z force is capped by the HC prediction at maximum allowed penetration |
| `variable_force` (VS-VF) | Derives the Z force from the HC prediction at desired penetration, bounded by configured force limits |

Outside detected contact, the desired QP force becomes zero and the minimum
spring remains available to follow pose commands. Linear X/Y wrench references
are used in contact. Moment references and positive-Z force references are
unsupported and return a controller error with zero effort. Orientation is
controlled through its fixed spring/damper.

Force tracking is indirect through the spring: choose a pose equilibrium that
provides a compression offset, as in the paper. At zero position error and zero
velocity, changing stiffness alone cannot generate a nonzero contact wrench.
There is no additive force feedforward term to replace that offset.

## Configuration

Merge this example into the actual robot's controller YAML. Replace all joint,
base/TCP/sensor names, topic and tissue/calibration values. The robot driver and
controller manager must already provide the model and hardware interfaces.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    adaptive_compliance_controller:
      type: adaptive_compliance_controller/AdaptiveComplianceController

adaptive_compliance_controller:
  ros__parameters:
    robot_base_link: base_link
    end_effector_link: tool0
    ft_sensor_ref_link: sensor_link
    ft_sensor_topic: /ft_sensor/wrench
    joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    command_current_configuration: false
    compensate_gravity: false
    compensate_coriolis: false
    compensate_dJdq: false
    nullspace_stiffness: 0.0
    hybrid_normal_force: false
    force_feedback_gain: 0.0
    command_timeout_s: 0.25
    sensor_timeout_s: 0.2
    max_contact_force: 4.0
    max_impedance_force: 30.0
    max_impedance_torque: 2.0
    delta_tau_max: 1.0
    sensor_tool_mass: 0.0
    sensor_tool_com: [0.0, 0.0, 0.0]
    sensor_bias: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sensor_force_sign: 1.0
    stiffness:
      rot_x: 50.0
      rot_y: 50.0
      rot_z: 50.0
    adaptive:
      mode: constant_force
      stiffness_min: [300.0, 300.0, 100.0]
      stiffness_max: [1000.0, 1000.0, 1000.0]
      damping: [30.0, 30.0, 20.0, 5.0, 5.0, 5.0]
      force_limit: [15.0, 15.0, 15.0]
      q: [3200.0, 3200.0, 3200.0]
      r: [0.00001, 0.00001, 0.00001]
      tank_initial: 0.5
      tank_min: 0.4
      tank_max: 1.0
      power_limit_w: 0.1
      contact_threshold_n: 0.5
      desired_penetration_m: 0.003
      max_penetration_m: 0.008
      max_period_s: 0.05
    tissue:
      map_directory: ''
      surface_z_m: 0.0
      kappa: 10000.0
      viscosity: 100.0
      beta: 1.35
```

These are illustrative values, not a tissue or robot calibration. FT mass/COM,
bias, sign and optional external tare use the
[force plugin's calibration contract](../force_impedance_controller/README.md#tool-load-compensation).
The adaptive plugin defaults its proportional force gain to zero. Nonzero force
feedback gain, hybrid mode, J-dot cancellation or a nullspace task are rejected:
they would add terms absent from this QP power budget. Rebuild the force plugin
when building this package, as it supplies the shared task-law extension point.

`adaptive.damping` is fixed and nonnegative, in base-axis order
`[x,y,z,rx,ry,rz]`. This keeps the dissipated-power term consistent in the QP
and applied task law. It is not the original code's stiffness-dependent damping
updated after solving with an old damping value. Translation is N/m and Ns/m;
rotation is Nm/rad and Nm s/rad. HC coefficient units depend on `beta`.
Energy is J, power is W, map lengths/depths are m. The parent translational
`stiffness.trans_*` values are not used by the adaptive task law.

## Commands and diagnostics

| Topic | Type | Contract |
| --- | --- | --- |
| `/<name>/target_frame` | `geometry_msgs/msg/PoseStamped` | TCP pose in the robot base |
| `/<name>/target_wrench` | `geometry_msgs/msg/WrenchStamped` | Linear tool-on-environment reference, at TCP when expressed in base axes |
| Configured `ft_sensor_topic` | `geometry_msgs/msg/WrenchStamped` | Raw environment-on-tool wrench at the sensor origin, sensor axes |
| `/<name>/adaptive_state` | `std_msgs/msg/Float64MultiArray` | Nonblocking diagnostic snapshot described below |

Pose/wrench frame conversion and timestamps follow the force plugin's contract.
With command expiry enabled, publish both target streams with fresh stamps,
including a zero wrench heartbeat in VS-VF. **FT freshness is mandatory even
when the wrench target is zero.** A nonzero target moment is not supported.

Diagnostic indices are:
`[tank_J, kx, ky, kz, reference_Fz, predicted_Fx, predicted_Fy, predicted_Fz,
kappa, viscosity, penetration_m, contact, QP_feasible, inhibited, variable_power_W]`.
The predicted wrench is the QP task result before final task/joint saturation;
it is not a measurement of delivered force.

## Tissue parameters and offline identification

With an empty map directory, configure a flat surface and scalar HC parameters.
Otherwise supply the same five whitespace-separated files used by the original:

- `x.txt`, `y.txt`: strictly increasing base-frame coordinates, at least two per axis;
- `z.txt`: surface height;
- `stiffness.txt`: HC elastic coefficient `kappa`;
- `damping.txt`: HC viscous coefficient.

Each grid has exactly `len(x)*len(y)` values, **x-major/y-minor**. Values are
bilinearly interpolated; malformed files, invalid coefficients or inconsistent
shapes fail configuration. Out-of-domain lookups inhibit the spring. There is
no built-in path, surface offset or extrapolation. A map is loaded on configure
and remains fixed; motion of the tissue requires external model registration.

The paper estimates the HC parameters offline and constructs smooth spatial
maps. A supplied identification helper fits a single palpation recording:

```bash
ros2 run adaptive_compliance_controller estimate_tissue.py contact_samples.csv --beta 1.35 --output fitted_parameters.json
```

CSV columns are `penetration_m,penetration_rate_m_s,contact_force_n`. Force is
positive compression, already corrected for bias, gravity and any relevant
inertial contribution. Samples must represent in-contact measurements with
positive penetration; dynamic excitation is required to separate elasticity
and viscosity. The helper performs nonnegative least squares and reports the
fit RMSE. It does not command palpation, zero the sensor or update the live
controller. Fit each location, then construct the calibrated map externally;
GP fitting and surface registration are not part of this package.

## Energy accounting, fallback and scope of passivity

For fixed references, the variable-stiffness power is
`P_var = (k-k_min) .* e dot v`. The QP enforces:

```
P_var <= sigma * v^T D v + (T - T_min)/dt
P_var <= sigma * v^T D v + power_limit
T_next = T + dt * (sigma * v^T D v - P_var)
```

The tank cannot be refilled to its minimum after failure. At its upper bound,
excess charge is discarded. Activation resets it to the configured initial
energy. Invalid periods (nonpositive or above `max_period_s`) fail the update
and clear effort commands instead of integrating invalid energy.

The three-variable diagonal QP is solved from source by enumerating at most
93 active sets of up to three constraints, using fixed-size matrices. No
precompiled qpOASES/MatLogger library or original experimental data is bundled.

On infeasible QP, missing/stale FT, command inhibition, overload or out-of-map
position, the spring is disabled and bounded dissipative damping remains.
The tank is frozen on these fallback cycles; dissipated energy is not credited.
The overload latch and deactivation behavior are inherited from the force
controller. Command increments still follow the hardware effort rate limit.

The energy constraints cover the **nominal translational task law** under the
paper's reference/model assumptions. Moving equilibrium references, rotation, fallback/re-engagement switches,
model mismatch, task saturation, actuator saturation/rate limits, sampling and
robot-driver dynamics require separate passivity/stability analysis. The tank
record is not a proof that the complete torque-controlled robot remains passive
under all those effects, nor a bound on actual tissue penetration or measured
contact force. Choose hardware limits and verify the applied controller on the
specific robot. This port has synthetic validation, with no hardware execution.

## Attribution and tests

The paper citation is listed before the anatomy-aware shared-control paper in
the [repository README](../README.md#attribution-and-citation).
`NOTICE` records the upstream commit and the changes in this torque port.
The BSD license is preserved in `LICENSE`.

```bash
colcon build --packages-up-to adaptive_compliance_controller --cmake-args -DBUILD_TESTING=ON -DLOGGING=OFF
colcon test --packages-select force_impedance_controller adaptive_compliance_controller
colcon test-result --verbose
```

Tests cover analytical QP solutions, HC behavior, map validation/interpolation,
discrete energy balance, tank depletion and power limits, offline estimation,
plugin loading, both control modes, load compensation before contact detection,
watchdogs, overload latching and direct joint effort output on synthetic hardware.
An optional independent SciPy comparison is provided in `test/compare_qp_scipy.py`
and takes the built `qp_probe` executable as its argument.
