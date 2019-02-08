# Source this file at the end of your .bashrc on the lab machines

export ROS_HOSTNAME=localhost # Optional, the name of this computer.
export ROS_MASTER_HOST=localhost # Used to inform us what robot we're connected to.
export ROS_MASTER_URI=http://localhost:11311 # The location of the ROS master.
export ROBOT=sim # The type of robot.

function my_ip() {
MY_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
echo $MY_IP
}

function setrobot() {
  if [[ "$1" == "sim" ]]; then
    export ROS_HOSTNAME=localhost;
    export ROS_MASTER_HOST=localhost;
    export ROS_MASTER_URI=http://localhost:11311;
    export ROBOT=sim;
  elif [[ "$1" == "racecar" ]]; then
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=$1$2;
    export ROS_MASTER_URI=http://172.16.77.$2:11311;
    export ROS_IP=`my_ip`;
  else
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=$1;
    export ROS_MASTER_URI=http://$1.cs.washington.edu:11311;
    export ROS_IP=`my_ip`;
  fi
}

