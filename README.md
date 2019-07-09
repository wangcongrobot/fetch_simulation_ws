# Training a arm to grasp a moving object
# run

roslaunch my_sawyer_openai_examples start_training_v2.launch

roslaunch haro_description my_move_block.launch

rostopic pub -1 /block/cmd_vel geometry_msgs/Twist "linear:
  x: 0.10
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.50"

