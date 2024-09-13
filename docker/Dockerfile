# Use the official ROS Noetic base image
FROM ros:noetic-ros-core

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Copy apt and python package files to the container
COPY apt_packages.txt /root/apt_packages.txt
COPY python_requirements.txt /root/python_requirements.txt

# Update package list and install necessary apt dependencies from the apt_packages.txt file
RUN apt-get update && xargs -a /root/apt_packages.txt apt-get install -y \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages from python_requirements.txt
RUN pip3 install -r /root/python_requirements.txt

# Install ROS tools like rosdep and catkin_tools
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep and update it
RUN rosdep init && rosdep update

# Create a catkin workspace
WORKDIR /root/catkin_ws

# Clone your Git repository into the src folder
RUN git clone https://github.com/your-username/your-ros-repo.git src

# Install ROS package dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the workspace setup for runtime
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set environment variables for ROS
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PACKAGE_PATH=/root/catkin_ws/src:/opt/ros/noetic/share
ENV ROS_IP=127.0.0.1

# Expose the necessary ROS ports (11311 is the default ROS master port)
EXPOSE 11311

# Copy your launch file to the container (if you have a launch file)
COPY launch_file.launch /root/catkin_ws/src/your_package/launch/launch_file.launch

# Set the default command to launch your ROS nodes
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch your_package launch_file.launch"]

