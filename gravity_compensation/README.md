# Gravity Compensation Controller

This controller implements a simple gravity compensation controller for a robot arm. It computes the required torque commands to counteract the gravitational forces acting on the robot's joints. For KUKA robots, it just send zero torque command, since the KUKA robots are already gravity compensated.

## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    gravity_compensation:
      type: gravity_compensation/GravityCompensation

    # More controller instances here
    # ...

/**/gravity_compensation:
  ros__parameters:
    end_effector_link: "lbr_link_ee"
    ft_sensor_ref_link: "lbr_link_ee" # Reference link for the force-torque sensor, if applicable
    robot_base_link: "lbr_link_0"
    compensate_gravity: false # for robots different than KUKA set to true
    compensate_coriolis: false # for robots different than KUKA set to true
    command_current_configuration: true # for KUKA set this to true, for other robots set to false
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    joints:
      - lbr_A1
      - lbr_A2
      - lbr_A3
      - lbr_A4
      - lbr_A5
      - lbr_A6
      - lbr_A7


# More controller specifications here
# ...

```

