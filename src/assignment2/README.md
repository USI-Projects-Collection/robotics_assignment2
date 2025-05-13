# RoboMaster
# Task implemented
- Open-Loop Controller
- Wall Detection and Alignment
- Wall Avoidance and Positioning

# Instructions
- Build the package
On a terminal run:
cd <path_to_root_of_the_package>
colcon build

- Source the package
On another terminal run:
cd <path_to_root_of_the_package>
source install/setup.zsh

**Task 1**
- Run the simulation
On a first terminal run:
cd <path_to_root_of_the_package>
pixi run coppelia

- Load the model
In coppelia -> File -> Load model -> select the model `src/robomaster_example/models/ros2Interface helper tool + clock.ttm`
Then run (Run symbol).

- On a second terminal run:
cd <path_to_root_of_the_package>
ros2 launch robomaster_ros ep.launch name:=/rm0

- On a third terminal run:
cd <path_to_root_of_the_package>
ros2 launch assignment2 controller.launch name:=/rm0

- Link to the video: https://drive.google.com/file/d/120rBB7-HCTBpsSvf9seu4iQ1YGjopTAy/view?usp=drive_link

**Task 2/3**
- Run the simulation
On a first terminal run:
cd <path_to_root_of_the_package>
pixi run coppelia

- Load the scene
In coppelia -> File -> Open scene -> select the scene `src/robomaster_example/scenes/robomaster-random-wall-scene.ttt`

- Load the model
In coppelia -> File -> Load model -> select the model `src/robomaster_example/models/ros2Interface helper tool + clock.ttm`

- On a second terminal run:
cd <path_to_root_of_the_package>
ros2 launch assignment2 ep_tof.launch name:=/rm0

- On a third terminal run:
cd <path_to_root_of_the_package>
ros2 launch assignment2 standard.launch name:=/rm0

- Link to the video: https://drive.google.com/file/d/1-l6Upyxq00uQgIKbRyeGvhQN9l0uFtkn/view?usp=drive_link