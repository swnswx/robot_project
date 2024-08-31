# robot_project

This project is designed to run on the following hardware:
- **Raspberry Pi 5**
- **Jetson Nano**

## Required Python Libraries

To run this project, you need to install the following Python libraries:

### Tracking
- `cvzone`
- `ultralytics`
- `scikit-image`
- `filterpy`
- `dill`

## Installation

To install all the required libraries, use the `requirements.txt` file. Make sure the file is in the root directory of the project, then run the following command:

```bash
pip install -r requirements.txt
```
## Running the Project

### On Raspberry Pi 5

To run the project on the Raspberry Pi 5, execute the following commands:

1. Open a terminal and source the setup file:
    ```bash
    source install/local_setup.bash
    ```
2. Run the lane detection node:
    ```bash
    ros2 run lane_detection_package lane_detection_node
    ```

### On Jetson Nano

To run the project on the Jetson Nano, use two terminals and execute the following commands:

- **Terminal 1:**
  1. Source the setup file:
     ```bash
     source install/local_setup.bash
     ```
  2. Run the object tracking node:
     ```bash
     ros2 run tracking_package object_tracking
     ```

- **Terminal 2:**
  1. Source the setup file again:
     ```bash
     source install/local_setup.bash
     ```
  2. Run the micro ROS bridge node:
     ```bash
     ros2 run micro_ros_bridge micro_ros_bridge_node
     ```

Make sure to execute these commands in the correct environment for each hardware platform.
