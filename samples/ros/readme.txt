Prerequisites:
  ROS

Install ROS Noetic

  >> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  >> sudo apt install curl # if you haven't already installed curl
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  >> sudo apt update
  
  >> sudo apt install ros-noetic-desktop-full

  >> echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  
  >> source ~/.bashrc

  >> sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
  >> sudo rosdep init
  
  >> rosdep update

Build and run application:

  >> catkin_make

4. To start the ROS application, run command:

  # ./run.sh

5. To stop ROS, please press Ctr + C in terminal 

