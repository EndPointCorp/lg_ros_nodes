# Basic docker file for running tests, lives in the root directory because
# it needs to add the entire project into the container
FROM ros:indigo
MAINTAINER Jacob Minshall <jacob@endpoint.com>

# install system dependencies
RUN apt-get update && \
      apt-get install -y g++ pep8 && \
      rm -rf /var/lib/apt/lists/*

# entrypoint
COPY scripts/docker_entrypoint.sh /ros_entrypoint.sh

# add galadmin user, being root isn't fun
RUN useradd -ms /bin/bash galadmin
RUN echo "galadmin   ALL=NOPASSWD: ALL" >> /etc/sudoers

USER galadmin

ENV PROJECT_ROOT /home/galadmin/catkin_ws
RUN mkdir -p $PROJECT_ROOT/catkin/src
RUN mkdir -p $PROJECT_ROOT/scripts

# test runner script & friends
COPY scripts/ $PROJECT_ROOT/scripts
COPY setup.cfg $PROJECT_ROOT/

CMD cd /home/galadmin/catkin_ws/catkin && \
    sudo cp -r /docker_nodes/* src/ && \
    sudo chown -R galadmin:galadmin /home/galadmin/ && \
    sudo apt-get update && \
    rosdep update && \
    sudo rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    catkin_make && \
    . devel/setup.sh && \
    cd .. && \
    ./scripts/test_runner.py
