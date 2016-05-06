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
RUN chmod 0755 /ros_entrypoint.sh

# add test user, being root isn't fun
ENV TEST_USER test_docker
ENV HOME /home/test_docker
RUN useradd -ms /bin/bash -d ${HOME} ${TEST_USER} \
 && echo "${TEST_USER} ALL=NOPASSWD: ALL" >> /etc/sudoers

USER $TEST_USER

ENV PROJECT_ROOT $HOME/catkin_ws
RUN mkdir -p $PROJECT_ROOT/catkin/src \
 && mkdir -p $PROJECT_ROOT/scripts

# test runner script & friends
COPY scripts/ $PROJECT_ROOT/scripts
COPY setup.cfg $PROJECT_ROOT/

CMD cd ${HOME}/catkin_ws/catkin && \
    sudo cp -r /docker_nodes/* src/ && \
    sudo chown -R ${TEST_USER}:${TEST_USER} ${HOME} && \
    sudo apt-get update && \
    rosdep update && \
    sudo rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    catkin_make && \
    . devel/setup.sh && \
    cd .. && \
    ./scripts/test_runner.py
