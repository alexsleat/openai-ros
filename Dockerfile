# base image for docker:
FROM osrf/ros:noetic-desktop

##########################################################################
# Install common applications:
##########################################################################

ENV DEBIAN_FRONTEND=noninteractive 
RUN apt-get update
RUN apt-get install -y wget curl tar python python3-pip nano
RUN apt-get install -y ros-noetic-catkin

RUN apt-get update --fix-missing

# Install ROS tools:
RUN pip3 install -U rosdep rosinstall_generator wstool rosinstall

# Install OpenAI:
RUN pip3 install openai

# Add ROS to bashrc:
RUN bash -c "echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc"


##########################################################################
# Robot installations:
##########################################################################


##########################################################################
# Set up your environment:
#
## Uncomment block under the title of the tools you want:
##########################################################################

# Add directories from the host, to the guest:
## WORKDIR = Guest directory (within the docker):
WORKDIR "/home/user/"   
## ADD [host_directory_path] [guest_directory_name]
# ADD test test/

COPY config.sh /home/user/
RUN bash -c "source /home/user/config.sh"
## @TODO below not working correctly:
WORKDIR "/home/user/code"

## Auto build your ROS workspace:
# RUN bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/ws; catkin_make'
# RUN bash -c "echo 'source /home/user/ws/devel/setup.bash' >> ~/.bashrc"
