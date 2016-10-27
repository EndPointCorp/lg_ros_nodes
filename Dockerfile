# Basic docker file for running tests, lives in the root directory because
# it needs to add the entire project into the container
FROM ros:indigo
MAINTAINER Jacob Minshall <jacob@endpoint.com>

# install system dependencies
RUN apt-get update && \
      apt-get install -y g++ pep8 cppcheck \
      python-pytest wget \
      xvfb x11-apps && \
      rm -rf /var/lib/apt/lists/*

RUN \
      wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add - && \
      echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google.list && \
      apt-get update && \
      apt-get install -y google-chrome-stable && \
      rm -rf /var/lib/apt/lists/*


# entrypoint
COPY scripts/docker_entrypoint.sh /ros_entrypoint.sh
RUN chmod 0755 /ros_entrypoint.sh

COPY scripts/docker_xvfb_add.sh /docker_xvfb_add.sh
RUN chmod 0755 /docker_xvfb_add.sh && sync
ENV DISPLAY :1

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
    /docker_xvfb_add.sh && \
    sudo chown -R ${TEST_USER}:${TEST_USER} ${HOME} && \
    sudo apt-get update && \
    rosdep update && \
    sudo rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    catkin_make && \
    . devel/setup.sh && \
    cd .. && \
    ./scripts/test_runner.py
