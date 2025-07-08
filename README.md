# KUKA CLIK Controller
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/humble.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/jazzy.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/rolling.yml/badge.svg)


This branch implements command the joint configuration to the [lbr_stack](https://github.com/idra-lab/lbr_fri_ros2_stack) KUKA hardware interface so it allows Cartesian and Joint Impedance Control as well as Joint Position Control depending on the command input chosen as `client_command_mode` in the hardware interface.

This controller is designed to be used with the KUKA FRI in `POSITION` command mode. To switch FRI Command Mode you need to update the following line in the `LBRServer.java` application, chosing between the two controller.
```
% for cartesian impedance control
control_mode_ = new CartesianImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);
```
```
% for cartesian impedance control
control_mode_ = new PositionControlMode();
```

|Supported Controller Options| FRI Command Mode   | FRI Controller Mode            |
|----------------------------|--------------------|--------------------------------|
|Kinematics Control          |`POSITION`          | `PositionControlMode`          |
|Joint Impedance Control     |`POSITION`          | `JointImpedanceControlMode`    |
|Cartesian Impedance Control |`POSITION`          | `CartesianImpedanceControlMode`|

Check out how to use this controller in our KUKA LBR example [here](https://github.com/idra-lab/kuka_impedance)!  
The code structure and some libraries have been taken from the repo [Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).


#### Parameters:
Below is an example `controller_manager.yaml` for a controller specific configuration.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    kuka_clik_controller:
      type: kuka_clik_controller/KukaClikController

    # More controller instances here
    # ...

/**/kuka_clik_controller:
  ros__parameters:
    max_linear_velocity: 0.3
    max_angular_velocity: 0.3
    end_effector_link: "lbr_link_ee"
    robot_base_link: "lbr_link_0"
    compliance_ref_link: "lbr_link_ee"
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
    nullspace_desired_configuration:
      - 0.0
      - 0.70
      - 0.0
      - -1.25
      - 0.0
      - 1.40
      - 0.0
    
    # More controllers here
```
