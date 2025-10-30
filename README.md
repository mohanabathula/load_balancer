# Performance Aware Load Balancer 
# Real - Time control
..................
____________________________
# CARLA (0.9.13) with ROS Bridge for Ubuntu Noetic

This guide explains how to run **CARLA 0.9.13** with the **CARLA ROS Bridge** on Ubuntu Noetic. Follow these steps to launch the CARLA simulator, start the ROS bridge, and control the ego vehicle manually.

---

## System Requirements

- **Ubuntu 20.04** or later
- **ROS Noetic** installed
- **CARLA 0.9.13** installed
- **Python 3.8** or later
- **CARLA ROS Bridge** installed

---

## Documentation Links

- **CARLA 0.9.13 Documentation**: [CARLA 0.9.13 Build Guide](https://carla.readthedocs.io/en/0.9.13/build_linux/)
- **CARLA ROS Bridge Documentation**: [ROS Bridge Docs](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/)
- **CARLA ROS Bridge GitHub Repository**: [GitHub - carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)

---

## Steps to Run CARLA with ROS Bridge

### 1. Launch CARLA Server

To begin, start the **CARLA Editor** and launch the CARLA simulator in play mode.
Login to VNC next follow the next steps.

1. Open a terminal and navigate to the **CARLA directory**:
    ```bash
    cd ~/carla-0.9.13
    ```

2. Launch CARLA:
    ```bash
    make launch
    ```

    This will start the CARLA and then play the simualtor then server will start, and the simulation will be ready to connect with ROS Bridge.

---

### 2. Launch CARLA ROS Bridge

Next, we will launch the **CARLA ROS Bridge** and spawn an example ego vehicle.

1. Open another terminal and navigate to the **CARLA ROS Bridge workspace**:
    ```bash
    cd ~/carla-ros-bridge/catkin_ws
    ```

2. Source the **workspace** to set up the ROS environment:
    ```bash
    source devel/setup.bash
    ```

3. Launch the **CARLA ROS Bridge** with an example ego vehicle:
    ```bash
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
    ```

    This command will launch the ROS Bridge and spawn an example ego vehicle in the CARLA simulation. The bridge will also expose relevant topics and services to interact with the simulation via ROS.

---

## 3.Manual Control of the Ego Vehicle

Once the CARLA simulation and ROS Bridge are running, you can control the ego vehicle manually using the **keyboard**.

### Keyboard Controls

| Key | Action                                      |
| --- | ------------------------------------------- |
| `W` | Accelerate / Move forward                   |
| `A` | Turn left                                   |
| `S` | Brake                                       |
| `D` | Turn right                                  |
| `Q` | Revese gear                                 |
| `B` | Toggle manual control mode (enable/disable) |

> If manual control is not responding, press **`B`** once to enable manual override mode.

---

## 4.Viewing Camera Streams (RGB, Depth, Semantic)

To visualize the camera streams published by CARLA, run:

```bash
rqt_image_view
```

Then select the topics from the dropdown list:

| Camera Type           | ROS Topic                                              |
| --------------------- | ------------------------------------------------------ |
| RGB Camera            | `/carla/ego_vehicle/rgb_front/image`                   |
| Depth Camera          | `/carla/ego_vehicle/depth_front/image`                 |
| Semantic Segmentation | `/carla/ego_vehicle/semantic_segmentation_front/image` |

---

## 5.Publish Vehicle Control Commands Manually (Example)

To command the vehicle to **take a left turn** using ROS:

```bash
rostopic pub /carla/ego_vehicle/vehicle_control_cmd_manual carla_msgs/CarlaEgoVehicleControl \
"{throttle: 0.3, steer: 0.5, brake: 0.0, hand_brake: false, reverse: false, gear: 0, manual_gear_shift: false}"
```

### Explanation of Values

| Field            | Meaning                                         |
| ---------------- | ----------------------------------------------- |
| `throttle: 0.3`  | 30% forward acceleration                        |
| `steer: 0.5`     | Turn left (+1.0 = full left, -1.0 = full right) |
| `brake: 0.0`     | No braking                                      |
| `reverse: false` | Drive forward                                   |

---

## Troubleshooting

- **CARLA is not responding**: Ensure that the CARLA server is running before launching the ROS Bridge.
- **Keyboard control not working**: Press `B` to toggle manual control mode if the vehicle doesn't respond.
- **Messages not loading**: Make sure to build your ROS workspace with `catkin_make` or `catkin build` and source it with `source devel/setup.bash`.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
