# Basic docker file for running tests, lives in the root directory because
# it needs to add the entire project into the container
FROM ros:indigo
MAINTAINER Jacob Minshall <jacob@endpoint.com>

# install system dependencies
RUN apt-get update && \
      apt-get install -y g++ pep8 cppcheck closure-linter \
      python-pytest wget \
      xvfb x11-apps && \
      rm -rf /var/lib/apt/lists/*

RUN \
      wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add - && \
      echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google.list && \
      apt-get update && \
      apt-get install -y google-chrome-stable git sudo && \
      rm -rf /var/lib/apt/lists/*

RUN \
     apt-get update && \
     sudo apt-get install -y ros-indigo-rosapi libudev-dev

# entrypoint for setup.bash
COPY scripts/docker_entrypoint.sh /ros_entrypoint.sh
RUN chmod 0755 /ros_entrypoint.sh

# Xvfb support - chrome preloading requires that
ENV DISPLAY :1

# add test user, being root isn't fun
ENV TEST_USER test_docker
ENV HOME /home/test_docker
RUN useradd -ms /bin/bash -d ${HOME} ${TEST_USER} \
 && echo "${TEST_USER} ALL=NOPASSWD: ALL" >> /etc/sudoers
USER $TEST_USER

# make dirs and check out repos
ENV PROJECT_ROOT $HOME/src/lg_ros_nodes
RUN mkdir -p $PROJECT_ROOT/src
# set up all dirs for catkin and test scripts
RUN rm -fr $HOME/src/lg_ros_nodes; git clone https://github.com/EndPointCorp/lg_ros_nodes.git $HOME/src/lg_ros_nodes
RUN rm -fr $HOME/src/appctl ; git clone https://github.com/EndPointCorp/appctl.git $HOME/src/appctl

#let's build ros nodes
RUN cd ${PROJECT_ROOT} && \
    /ros_entrypoint.sh ./scripts/init_workspace -a $HOME/src/appctl && \
    chmod +x ./scripts/docker_xvfb_add.sh && \
    sudo chown -R ${TEST_USER}:${TEST_USER} ${HOME} && \
    ./scripts/docker_xvfb_add.sh && \
    cd ${PROJECT_ROOT}/catkin/ && \
    sudo apt-get update && \
    rosdep update # && \
    sudo rosdep install --from-paths src --ignore-src --rosdistro indigo -y

RUN cd ${PROJECT_ROOT}/catkin && /ros_entrypoint.sh catkin_make

# by default let's run tests
CMD cd ${PROJECT_ROOT}/catkin && \
    . devel/setup.sh && \
    cd ${PROJECT_ROOT} && \
    ./scripts/test_runner.py
