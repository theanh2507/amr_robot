# use the official ros2 humble base image
FROM ros:humble 

# install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists*

# create workspace 
WORKDIR /ros2_ws

# copy package source code into the workspace (copy thu muc src tu thu muc nguon vao src cua docker container)
COPY ./src ./src

# source the ros2 enviroment 
SHELL [ "/bin/bash", "-c"]

# build the workspace
RUN source /opt/ros/humble/setup.bash && colcon build

# source the built workspace and set the entrypoint to tun package
ENTRYPOINT [ "/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/set.bash"]

# start the app using serve command
# CMD ["serve", "-s", "build"]
