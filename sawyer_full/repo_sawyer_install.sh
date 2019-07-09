# These are the commands used to download the original code
cd ~/simulation_ws/src
git clone https://github.com/RethinkRobotics/sawyer_simulator.git
cd ~/simulation_ws/src
wstool init .
wstool merge sawyer_simulator/sawyer_simulator.rosinstall
wstool update
source /opt/ros/kinetic/setup.bash
cd ~/simulation_ws
# Install dependencies
./sawyer_dependencies_install.sh

catkin_make
source devel/setup.bash
# Test it moves the joints when pressing keys 1,q,2,w,3,e,4,r,5,t,6,y,7,u
rosrun intera_examples joint_position_keyboard.py
