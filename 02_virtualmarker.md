# Virtual Kinesthetic Teaching for Bimanual Telemanipulation

This repo is for the demo: https://youtu.be/qV9ol3f4JtY

## Installation

The entire system needs a ROS computer that runs a robotic manipulator (slave) and a Windows computer that runs VR interface (master). 

(1) ROS side
- `universal_robot` and `ur_modern_driver`
  - This repo uses a UR5 robot as the slave robot. But, this repo can be easily adapted to any other robotic manipulators as long as they can be running with `ros_control`. 
  - Install `universal_robot` using `apt`, as recommended in https://github.com/ros-industrial/universal_robot: 
    "NOTE: please prefer using the binary release (see previous section) over building from source where possible. Source installs will not be automatically updated by new package releases and require more work to setup."
    ```
    sudo apt-get install ros-$ROS_DISTRO-universal-robot
    ```
    - replace `$ROS_DISTRO` with `hydro`, `indigo` or `kinetic`, depending on which ROS version you have installed.
    
  - Install `ur_modern_driver` (https://github.com/inmo-jang/ur_modern_driver)
    ```
    git clone https://github.com/inmo-jang/ur_modern_driver.git -b kinetic-devel
    ```   
    - replace `kinetic-devel` with another depending on which ROS version you have installed.

- `ros_control` (http://gazebosim.org/tutorials/?tut=ros_control)


- `relaxed_ik`
  - Install as follows:
    ```
    git clone https://github.com/inmo-jang/relaxed_ik-origin.git -b -dev
    ```
    If the above cloning doesn't work, then run the following instead:
    ```
    git clone https://github.com/inmo-jang/relaxed_ik-origin.git
    git checkout dev
    ```
  - You also need to set some parameters in `start_here.py` in `relaxed_ik` package.
  - NOTE: You should rename the source folder `relaxed_ik-origin` to `relaxed_ik`
    
- Gripper (Robotiq 3-finger Gripper) (https://github.com/inmo-jang/robotiq)
  - Install
    ```
    git clone https://github.com/inmo-jang/robotiq.git
    ```
    
- rain_ros (https://github.com/inmo-jang/rain_ros)
  - This package includes some gazebo robot models and ros_sharp to connect to Unity. 
  
    ```
    git clone https://github.com/inmo-jang/rain_ros.git
    ```


(2) Windows side

- Unity Application (https://github.com/inmo-jang/unity_virtualmarker_teleop)
  - TODO: This needs to be properly uploaded.
  
## Execution

(1) Bring the robot: 
  * UR5
    - Connect to a UR5: 
       - Real Robot: `roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=172.22.22.2`
       - Gazebo: `roslaunch rain_gazebo ur5_robotiq.launch gripper:=robotiq_3f` (You should click the play button afterwards)
       
    - Switch ros_control:
      ```
      rosservice call /controller_manager/switch_controller "start_controllers:
      - 'joint_group_position_controller'
      stop_controllers:
      - 'pos_based_pos_traj_controller'
      strictness: 2"
      ```
    - Load the robot info file (automatically accessing to `/joint_states` and use it as `starting_config`): 
      ```
      roslaunch relaxed_ik load_info_file.launch
      ```
    - Run RelaxedIK solver: `roslaunch relaxed_ik relaxed_ik_python.launch`
    
    - Rviz (Rviz takes `\joint_states` from the robot and provides new joint position command via `/joint_group_position_controller/command`)
      ```
      roslaunch relaxed_ik rviz_viewer_for_hri_test.launch
      rosrun relaxed_ik marker_ikgoal_driver_for_hri_test.py
      ```
     - Using the marker in Rviz, move the arm slightly, which activates the rest of the process (Needs confirmation).

  * Gripper (Robotiq 3-finger Gripper)
    - Connect to the gripper (Skip for Gazebo test):
       - `rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11`
    - Run the controller:
       - `rosrun robotiq_s_model_control SModelController_unity.py [gazebo or real]` - Command from Unity ("Float32")
       - (For keyboard testing): `rosrun robotiq_s_model_control SModelController.py [gazebo or real]`
    - Run the joint_state_publisher: This will publish `/gripper_joint_states`
       - `rosrun robotiq_joint_state_publisher s_model_joint_states _prefix:=r_`
       - (For Gazebo) `rosrun robotiq_joint_state_publisher s_model_joint_states _prefix:=r_ _model:=gazebo`
       
  
(2) Open a socket to Unity: `roslaunch rain_unity ur5_robotiq_unity_real.launch`

(3) Run a Unity Application:
  * Virtual Marker: After running the app, you should push "m" (which initialises the market position) and "p" (which starts publishing)       
