

# install
```bash
sudo su
pip install keyboard
```


# calibration (need root)
```bash
cd ~/catkin_ws
sudo su
source /opt/ros/melodic/setup.bash
source devel/setup.bash

## Calibration for grippers
rosrun senseglove_sy_gripper glove_gripper_caliration.py

## Calibration for robotic hands
rosrun senseglove_sy_gripper hand_cali.py
```

# run

```bash
## Run grippers
rosrun senseglove_sy_gripper run_gripper.py
## Run robotic hands
roslaunch senseglove_sy_gripper both_hands.launch

```
