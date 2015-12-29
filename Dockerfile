# Basic docker file for running tests, lives in the root directory because
# it needs to add the entire project into the container
FROM ros:indigo
MAINTAINER Jacob Minshall <jacob@endpoint.com>

# install system dependencies
RUN apt-get update && \
      apt-get install -y g++ pep8 && \
      rm -rf /var/lib/apt/lists/*

# add galadmin user, being root isn't fun
RUN useradd -ms /bin/bash galadmin

ENV PROJECT_ROOT /home/galadmin/catkin_ws
RUN mkdir -p $PROJECT_ROOT/catkin
RUN mkdir -p $PROJECT_ROOT/scripts
ADD docker_nodes/ $PROJECT_ROOT/catkin/src
RUN chown -R galadmin:galadmin $PROJECT_ROOT

# set up project dependencies
RUN cd $PROJECT_ROOT/catkin && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    rm -rf /var/lib/apt/lists/*

# build the project
USER galadmin
RUN bash -c "cd $PROJECT_ROOT/catkin; . /opt/ros/indigo/setup.bash; catkin_make;"

# entrypoint
ADD scripts/docker_entrypoint.sh /ros_entrypoint.sh
# test runner script & friends
ADD scripts/ $PROJECT_ROOT/scripts
ADD setup.cfg $PROJECT_ROOT/

CMD cd /home/galadmin/catkin_ws/catkin && \
    catkin_make && \
    cd .. && \
    ./scripts/test_runner.py
