# Build from https://github.com/osrf/docker_images/tree/master/ros/indigo/indigo-ros-base
FROM ros:indigo
MAINTAINER Dmitry Kiselev <dmitry@endpoint.com>

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV DEBIAN_FRONTEND=noninteractive

# Install misc system dependencies.
RUN \
  apt-get update && \
  apt-get -y install --no-install-recommends \
    bc \
    lsb-core \
    x-window-system \
    dbus-x11 \
    dconf-cli \
    wget \
    curl \
    tmux \
    apt-transport-https \
    libglu1-mesa \
    socat \
    libnss3-tools \
    gir1.2-atspi-2.0 \
    binutils \
    mesa-utils \
    mesa-utils-extra \
    module-init-tools \
    gdebi-core \
    tar \
    libfreeimage3 \
    xdg-utils \
    onboard \
    git \
    libexif12 \
    libexif-dev

# Install more dependencies from the End Point Portal repository and upstream.
RUN \
  echo "deb https://packages.endpoint.com/apt/lg trusty main" > /etc/apt/sources.list.d/endpoint-lg.list && \
  echo "deb [arch=amd64] http://portapt.galaxy.endpoint.com:5831/portal-stable trusty main" > /etc/apt/sources.list.d/portal-stable.list && \
  apt-get update && \
  apt-get -y --no-install-recommends install \
    spacenavd \
    g++ \
    ros-indigo-spacenav-node \
    ros-indigo-rosbridge-suite \
    ros-indigo-rosbridge-server \
  && rm -rf /var/lib/apt/lists/*

# Install Chrome and the Portal ROS packages from the End Point Portal repository and upstream.
RUN \
  echo "deb [arch=amd64] http://portapt.galaxy.endpoint.com:5831/lg-ros-indigo-1.1.36 trusty main" > /etc/apt/sources.list.d/lg-ros.list && \
  echo "deb [arch=amd64] http://portapt.galaxy.endpoint.com:5831/lg-sv-nonfree-indigo-0.0.9 trusty main" > /etc/apt/sources.list.d/lg-sv-nonfree.list && \
  echo "deb [arch=amd64] http://portapt.galaxy.endpoint.com:5831/appctl-indigo-1.0.10 trusty main" > /etc/apt/sources.list.d/appctl.list && \
  echo "deb [arch=amd64] http://portapt.galaxy.endpoint.com:5831/google-chrome-stable-47 trusty main" > /etc/apt/sources.list.d/google-chrome.list && \
  apt-get update && \
  apt-get -y --force-yes --no-install-recommends install \
    ros-indigo-liquidgalaxy \
    ros-indigo-lg-sv-nonfree \
    google-chrome-stable \
  && rm -rf /var/lib/apt/lists/*

  # Install GE
  WORKDIR /tmp
  RUN     wget -q https://dl.google.com/dl/earth/client/current/google-earth-stable_current_amd64.deb ;\
          gdebi -n google-earth-stable_current_amd64.deb ;\
          rm google-earth-stable_current_amd64.deb

  # Patch for google earth from amirpli to fix some bugs in google earth qt libs
  # Without this patch, google earth can suddenly crash without a helpful error message.
  # See https://productforums.google.com/forum/?fromgroups=#!category-topic/earth/linux/_h4t6SpY_II%5B1-25-false%5D
  # and Readme-file https://docs.google.com/file/d/0B2F__nkihfiNMDlaQVoxNVVlaUk/edit?pli=1 for details

  RUN     mkdir -p /opt/google/earth/free ;\
          touch /usr/bin/google-earth ;\
          cd /opt/google/earth ;\
          cp -a /opt/google/earth/free /opt/google/earth/free.newlibs ;\
          wget -q -P /opt/google/earth/free.newlibs \
            https://github.com/mviereck/dockerfile-x11docker-google-earth/releases/download/v0.3.0-alpha/ge7.1.1.1580-0.x86_64-new-qt-libs-debian7-ubuntu12.tar.xz ;\
          tar xvf /opt/google/earth/free.newlibs/ge7.1.1.1580-0.x86_64-new-qt-libs-debian7-ubuntu12.tar.xz ;\
          mv /usr/bin/google-earth /usr/bin/google-earth.old ;\
          ln -s /opt/google/earth/free.newlibs/googleearth /usr/bin/google-earth

#Use bash instead of sh
RUN     rm /bin/sh && ln -s /bin/bash /bin/sh

# Create lg user, add ROS env variables for lg and root
# Now, with sh changed by bash setup.bash sourced for
# build script command
RUN useradd -ms /bin/bash lg && \
    chown -R lg:lg /home/lg ;\
    echo "source /opt/ros/indigo/setup.bash" >> /home/lg/.bashrc ;\
    echo "source /opt/ros/indigo/setup.bash" >> /root/.bashrc

USER    lg
WORKDIR /home/lg

RUN	    mkdir /home/lg/src && cd /home/lg/src ;\
        git clone git://github.com/EndPointCorp/lg_ros_nodes.git ;\
	    git clone git://github.com/EndPointCorp/appctl.git ;\
        cd /home/lg/src/lg_ros_nodes && \
        git checkout features/docker-image

USER    root
WORKDIR /home/lg/src/lg_ros_nodes
RUN     source /opt/ros/indigo/setup.bash ;\
        rosdep init ;\
        rosdep update ;\
        ./scripts/init_workspace --appctl /home/lg/src/appctl ;\
        cd /home/lg/src/lg_ros_nodes/catkin ;\
        rosdep install --from-paths src --ignore-src --rosdistro indigo -y ;\
        catkin_make ;\
        source /home/lg/src/lg_ros_nodes/catkin/devel/setup.bash ;\
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo install

# Start the application as configured.
ENTRYPOINT tmux
