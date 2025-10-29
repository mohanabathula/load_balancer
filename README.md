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

    This will start the CARLA and the play the simualtor then server will start, and the simulation will be ready to connect with ROS Bridge.

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

### 3. Manual Control of the Vehicle

Once the CARLA simulation and ROS Bridge are running, you can control the ego vehicle manually using the **keyboard**.

- **To drive the vehicle**, use the following keys:
    - `W` - Move forward
    - `A` - Turn left
    - `S` - Move backward
    - `D` - Turn right
    - `Q` - Turn left more sharply
    - `E` - Turn right more sharply

- **If manual control does not work**, press `B` to enable manual control mode for the ego vehicle.

---

## Troubleshooting

- **CARLA is not responding**: Ensure that the CARLA server is running before launching the ROS Bridge.
- **Keyboard control not working**: Press `B` to toggle manual control mode if the vehicle doesn't respond.
- **Messages not loading**: Make sure to build your ROS workspace with `catkin_make` or `catkin build` and source it with `source devel/setup.bash`.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
