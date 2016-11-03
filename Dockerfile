# Basic docker file for running tests, lives in the root directory because
# it needs to add the entire project into the container
FROM ubuntu:14.04.3
MAINTAINER Jacob Minshall <jacob@endpoint.com>

# ENV vars to override for build or test
# Xvfb -> :1, native Xorg -> :0
ENV DISPLAY :1
ENV TEST_USER galadmin
ENV HOME /home/${TEST_USER}
ENV PROJECT_ROOT $HOME/src/lg_ros_nodes
ENV APPCTL_TAG 1.1.1
ENV NVIDIA_DRIVER_VERSION 352.41
ENV NVIDIA_DRIVER_URL http://us.download.nvidia.com/XFree86/Linux-x86_64/${NVIDIA_DRIVER_VERSION}/NVIDIA-Linux-x86_64-${NVIDIA_DRIVER_VERSION}.run

# entrypoint for setup.bash
COPY scripts/docker_entrypoint.sh /ros_entrypoint.sh
RUN chmod 0755 /ros_entrypoint.sh

RUN \
      sudo apt-get update -y && sudo apt-get install -y wget && \
      echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list && \
      echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list && \
      echo "deb http://dl.google.com/linux/earth/deb/ stable main" > /etc/apt/sources.list.d/google-earth.list &&\
      apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 5523BAEEB01FA116 && \
      wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add - && \
      apt-get update

RUN \
      apt-get install -y g++ pep8 cppcheck closure-linter \
      python-pytest wget \
      git sudo \
      curl tmux git \
      xvfb x11-apps \
      x-window-system binutils \
      mesa-utils mesa-utils-extra \
      module-init-tools gdebi-core \
      lsb-core tar libfreeimage3

RUN \
      apt-get install -y ros-indigo-rosapi libudev-dev \
      ros-indigo-ros-base ros-indigo-rosbridge-server

RUN \
      apt-get install -y google-chrome-stable google-chrome-beta google-chrome-unstable

RUN \
      apt-get install -y --no-install-recommends xdg-utils && \
      rm -rf /var/lib/apt/lists/*


#NVIDIA driver
RUN wget ${NVIDIA_DRIVER_URL} -O /tmp/nvidia-driver.run
RUN sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module ;\
    rm /tmp/nvidia-driver.run

# Install GE
# Patch for google earth from amirpli to fix some bugs in google earth qt libs
# Without this patch, google earth can suddenly crash without a helpful error message.
# See https://productforums.google.com/forum/?fromgroups=#!category-topic/earth/linux/_h4t6SpY_II%5B1-25-false%5D
# and Readme-file https://docs.google.com/file/d/0B2F__nkihfiNMDlaQVoxNVVlaUk/edit?pli=1 for details
WORKDIR /tmp
RUN  \
      wget -q https://dl.google.com/dl/earth/client/current/google-earth-stable_current_amd64.deb ;\
      gdebi -n google-earth-stable_current_amd64.deb ;\
      rm google-earth-stable_current_amd64.deb && \
      mkdir -p /opt/google/earth/free ;\
      touch /usr/bin/google-earth ;\
      cd /opt/google/earth ;\
      cp -a /opt/google/earth/free /opt/google/earth/free.newlibs ;\
      wget -q -P /opt/google/earth/free.newlibs \
        https://github.com/mviereck/dockerfile-x11docker-google-earth/releases/download/v0.3.0-alpha/ge7.1.1.1580-0.x86_64-new-qt-libs-debian7-ubuntu12.tar.xz ;\
      tar xvf /opt/google/earth/free.newlibs/ge7.1.1.1580-0.x86_64-new-qt-libs-debian7-ubuntu12.tar.xz ;\
      mv /usr/bin/google-earth /usr/bin/google-earth.old ;\
      ln -s /opt/google/earth/free.newlibs/googleearth /usr/bin/google-earth

# add root user
USER  root

# add galadmin user for tests and production
RUN \
      useradd -ms /bin/bash galadmin && \
      echo "galadmin   ALL=NOPASSWD: ALL" >> /etc/sudoers && \
	    mkdir -p /home/galadmin/src ;\
      echo "source /opt/ros/indigo/setup.bash" >> /root/.bashrc ;\
      echo "source /opt/ros/indigo/setup.bash" >> /home/galadmin/.bashrc ;\
      mv /bin/sh /bin/sh.bak && ln -s /bin/bash /bin/sh


# make dirs and check out repos
RUN mkdir -p $PROJECT_ROOT/src

# checkout appctl as a dependency
RUN rm -fr $HOME/src/appctl ; git clone --branch ${APPCTL_TAG} https://github.com/EndPointCorp/appctl.git $HOME/src/appctl


# copy all the ros nodes to source dir
COPY ./ ${PROJECT_ROOT}

#build ROS nodes
RUN \
    cd ${PROJECT_ROOT} && \
    source /opt/ros/indigo/setup.bash && \
    /ros_entrypoint.sh ./scripts/init_workspace -a $HOME/src/appctl && \
    ./scripts/docker_xvfb_add.sh && \
    cd ${PROJECT_ROOT}/catkin/ && \
    rosdep init &&\
    rosdep update && \
    sudo rosdep install \
        --from-paths /home/galadmin/src/lg_ros_nodes/catkin/src \
        --ignore-src \
        --rosdistro indigo \
        -y ;\
    catkin_make ;\
    catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo install;\
    source /home/galadmin/src/lg_ros_nodes/catkin/devel/setup.bash && \
    sudo chown -R ${TEST_USER}:${TEST_USER} ${HOME}

# by default let's run tests
CMD cd ${PROJECT_ROOT}/catkin && \
    . devel/setup.sh && \
    cd ${PROJECT_ROOT} && \
    ./scripts/test_runner.py
