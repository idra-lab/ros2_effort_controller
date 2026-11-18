# Force Impedance Controller

A `ros2_control` effort plugin combining Cartesian spring-damper impedance with
wrist force/torque feedback. Sensor load compensation runs in the controller
before feedback. An optional hybrid mode regulates force along the commanded
force direction while retaining pose impedance in the tangential directions
and orientation.

Plugin type: `force_impedance_controller/ForceImpedanceController`.
For pose-only impedance, use the separate
[`CartesianImpedanceController`](../cartesian_impedance_controller/README.md).

## Configuration

Merge this example into the actual robot controller config. Replace joint,
base, TCP and FT link names, sensor topic, and calibration values with those
of the hardware. Supply the robot description as explained in the
[repository README](../README.md#configure-the-robot).

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    force_impedance_controller:
      type: force_impedance_controller/ForceImpedanceController

force_impedance_controller:
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
    stiffness:
      trans_x: 500.0
      trans_y: 500.0
      trans_z: 500.0
      rot_x: 50.0
      rot_y: 50.0
      rot_z: 50.0
    hybrid_normal_force: true
    force_feedback_gain: 1.0
    command_timeout_s: 0.25
    sensor_timeout_s: 0.2
    max_contact_force: 4.0
    max_impedance_force: 10.0
    max_impedance_torque: 2.0
    delta_tau_max: 1.0
    task_inertia_cutoff: 0.000001
    nullspace_stiffness: 0.0
    # Illustrative calibration, not measured hardware data:
    sensor_tool_mass: 0.2
    sensor_tool_com: [0.0, 0.0, 0.05]
    sensor_bias: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sensor_force_sign: 1.0
    # For an externally tared sensor only, specify sensor->base orientation
    # at zeroing as [x, y, z, w], e.g. sensor_tare_orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

`ft_sensor_ref_link` must be a link in the configured base-to-TCP chain.
The example enables hybrid control and command expiry; the plugin defaults
are `hybrid_normal_force: false` and `command_timeout_s: 0.0`.
The generic mass default is zero. Reload/reconfigure after changing calibration;
parameters are not updated live.

## Topics, frames and signs

| Input | Message | Contract |
| --- | --- | --- |
| `/<controller_name>/target_frame` | `geometry_msgs/msg/PoseStamped` | Desired TCP pose in `robot_base_link` |
| `/<controller_name>/target_wrench` | `geometry_msgs/msg/WrenchStamped` | Desired tool-on-environment wrench, using the origin rules below |
| `ft_sensor_topic` | `geometry_msgs/msg/WrenchStamped` | Raw environment-on-tool measurement at the FT origin, in FT axes |

The sensor subscription uses `SensorDataQoS`, including compatibility with
best-effort sensor publishers. An empty `ft_sensor_topic` selects
`<controller_name>/ft_sensor_wrench`; it may be remapped to the driver's topic.
The sensor message must have `header.frame_id == ft_sensor_ref_link`.

Target wrench origin rules:

- With `frame_id == robot_base_link`, the wrench is already **at the TCP
  origin**, expressed in base axes. It is not a wrench about the base origin.
- With another allowed chain frame, the wrench is about that frame's origin,
  in its axes. The controller rotates it and shifts the moment to the TCP:
  `moment_tcp = R * moment_source + (p_source - p_tcp) cross force_base`.

The measured contact wrench is rotated and shifted from the FT origin to the
TCP after load compensation. A sign parameter normalizes reversed device
conventions. In a balanced contact, a target force of `+2 N` along base Z
corresponds to a measured reaction of `-2 N` along the same axis; the feedback
error is `target + measured`.

Use finite values, a nonzero pose quaternion and current ROS timestamps.
Quaternions are normalized; invalid messages or unsupported frames are ignored.
Both target streams must be refreshed before `command_timeout_s` when it is
positive. Age checks use receive time and ROS message time; all publishers must
use the same ROS clock domain as the controller. Messages more than 50 ms in
the future fail the enabled timestamp check.

## Impedance and force feedback

In full-impedance mode (`hybrid_normal_force: false`), all six spring directions
remain active. For a nonzero target wrench, the task command is:

```
W_cmd = K e - D J q_dot + W_target + k_f (W_target + W_contact)
tau   = J^T W_cmd + model_compensation + nullspace_torque
```

Stiffness axes follow the current TCP frame. Translational stiffness is N/m;
rotational stiffness is Nm/rad. Damping is computed from the regularized task
inertia. A zero target wrench disables the force-feedback term; it does not
request active regulation of zero contact force.

In hybrid mode, a nonzero target force defines a unit direction `n` in base
axes. The translational spring is projected onto the plane perpendicular to
`n`; normal damping is retained. Force feedback corrects only the component
along `n`. Rotational impedance remains active, and commanded moments are
feedforward terms without measured-moment feedback in this mode. If the target
contains only torque and no force, the full-mode wrench feedback applies.

The complete task force and moment, including optional `-Lambda J_dot q_dot`
compensation, are capped by `max_impedance_force` and `max_impedance_torque`.
Model compensation and the optional postural task are added separately, before
joint effort and increment limiting by the base controller.

## Tool load compensation

Configure the part of the tool downstream of the sensor:

| Parameter | Meaning | Default |
| --- | --- | --- |
| `sensor_tool_mass` | Downstream mass in kg | `0.0` |
| `sensor_tool_com` | FT origin to tool COM, in FT axes, metres | `[0, 0, 0]` |
| `sensor_bias` | Six device offsets `[Fx, Fy, Fz, Mx, My, Mz]`, N and Nm | Six zeros |
| `sensor_force_sign` | Normalize sensor convention to environment-on-tool | `1.0`, or `-1.0` |
| `sensor_tare_orientation_xyzw` | Sensor-to-base quaternion at an external zeroing pose | Empty: untared stream |

Let `R` rotate sensor axes into base axes. Gravity is assumed to be
`[0, 0, -9.80665] m/s^2` in `robot_base_link`; the base Z axis must therefore
point upward. Before transforming the wrench, the controller computes:

```
g_sensor = R^T [0, 0, -mass * 9.80665]
load     = g_sensor - g_sensor_at_tare
W_contact_sensor = sign * (W_raw - bias) - [load; COM cross load]
```

For an untared stream, `g_sensor_at_tare` is zero. For a stream zeroed externally
with the tool mounted, provide the actual sensor orientation at zeroing; its
gravity vector is subtracted from the current load. Activation does not
perform an automatic tare. Bias is subtracted before sign normalization.

If the driver already fully compensates tool gravity, use mass zero and avoid
compensating that load twice. Sensor-load subtraction and robot joint gravity
compensation are separate operations; configure each consistently with the
driver and robot model.

## Watchdogs and lifecycle

| Parameter | Behavior | Default |
| --- | --- | --- |
| `command_timeout_s` | Maximum receive/stamp age for both target streams; zero disables command expiry | `0.0 s` |
| `sensor_timeout_s` | Maximum FT receive/stamp age required for a nonzero force or torque target | `0.2 s` |
| `max_contact_force` | Norm of compensated contact force that latches an overload fault when exceeded | `4.0 N` |
| `force_feedback_gain` | Nonnegative proportional force-error gain | `1.0` |

Activation sets the pose equilibrium to the measured pose, clears the overload
latch and ignores inputs received before activation. A stale command, required
but stale FT sample, or latched overload clears the target wrench, sets the
spring equilibrium to the current pose and inhibits the postural task. Damping
and configured model compensation remain active.

Disabling command expiry does not disable sensor expiry. Fresh FT samples are
checked for overload even with a zero target. A detected overload remains
latched until deactivation/reactivation. Zero-effort writes on deactivation or
invalid state/model outputs are not a hardware brake or a gravity-hold command.

## Numerical behavior and tests

`task_inertia_cutoff` floors eigenvalues of `J M^-1 J^T` for modal damping; the
postural projector uses a truncated pseudoinverse. The cutoff is in the existing
unscaled linear/angular coordinates. Regularization protects the calculation
without restoring motion directions lost at a singularity.

`delta_tau_max` is a joint torque increment per update, not Nm/s. Logging is OFF
by default. The compatibility parameter `hand_frame_control` does not select a
frame; the message's `frame_id` does. Dynamic allocations and driver scheduling
still need profiling at the hardware update rate.

Build and test as described in the [repository README](../README.md#build).
`test/controller_lifecycle_test.cpp` checks synthetic interfaces and sensor
calibration, including compensation before overload detection.
`test/plugin_separation_test.cpp` discovers and loads both Cartesian plugins in
one process. Shared mathematical checks are in
`../effort_controller_base/test/control_math_test.cpp`. These tests do not
exercise a physical robot or certify closed-loop hardware behavior.
