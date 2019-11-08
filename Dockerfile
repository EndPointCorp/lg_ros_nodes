ARG UBUNTU_RELEASE=bionic
FROM ubuntu:${UBUNTU_RELEASE}
ARG UBUNTU_RELEASE

# prevent interactive prompts during build
ENV DEBIAN_FRONTEND noninteractive

# project settings
ENV PROJECT_ROOT $HOME/src/lg_ros_nodes
ENV ROS_DISTRO melodic

# Env for nvidia-docker2/nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

# entrypoint for ROS setup.bash
COPY scripts/docker_entrypoint.sh /ros_entrypoint.sh
RUN chmod 0755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# pre-install some tools for installing further dependencies
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    wget \
 && rm -rf /var/lib/apt/lists/*

# install system dependencies and tools not tracked in rosdep
RUN \
  echo "deb http://packages.ros.org/ros/ubuntu ${UBUNTU_RELEASE} main" > /etc/apt/sources.list.d/ros-latest.list && \
  echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list && \
  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
  wget --no-check-certificate -q -O /tmp/key.pub https://dl-ssl.google.com/linux/linux_signing_key.pub && apt-key add /tmp/key.pub && rm /tmp/key.pub && \
  apt-key update && \
  apt-get update && \
  apt-get install -y --no-install-recommends \
    automake autoconf libtool \
    g++ pycodestyle cppcheck \
    python-pytest wget \
    python-gst-1.0 \
    python-pip \
    python-setuptools \
    python3-pip \
    python3-setuptools \
    python3-defusedxml \
    python3-nose \
    python3-pil \
    python3-pytest \
    python3-netifaces \
    python3-serial \
    python3-tornado \
    git sudo \
    curl tmux git \
    xvfb x11-apps \
    x-window-system binutils \
    pulseaudio \
    mesa-utils mesa-utils-extra \
    module-init-tools gdebi-core \
    libxext-dev \
    lsb-core tar libfreeimage3 \
    ros-$ROS_DISTRO-rosapi libudev-dev \
    ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-web-video-server \
    ros-$ROS_DISTRO-spacenav-node spacenavd \
    google-chrome-stable google-chrome-beta google-chrome-unstable \
    awesome xdg-utils \
    gstreamer1.0-alsa \
 && rm -rf /var/lib/apt/lists/*

# Install NodeJS and test dependencies
RUN curl -sL https://deb.nodesource.com/setup_10.x | bash - \
 && apt-get install -y nodejs \
 && npm install -g eslint \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir python-coveralls \
 && pip3 install --no-cache-dir \
    wheel \
    rospkg \
    catkin_pkg \
    evdev \
    bson \
    pyinotify \
    catkin_tools \
    empy \
    pycrypto \
    gnupg

# Install GE
ENV GOOGLE_EARTH_VERSION ec_7.3.0.3832_64
ENV EARTH_PKG_URL https://roscoe-assets.galaxy.endpoint.com:443/google-earth/google-earth-stable_${GOOGLE_EARTH_VERSION}.deb
RUN mkdir -p /tmp/GE \
 && cd /tmp/GE \
 && wget $EARTH_PKG_URL \
 && dpkg -i $( basename $EARTH_PKG_URL ) \
 && rm $( basename $EARTH_PKG_URL ) \
 && if [ -f "/opt/google/earth/free/libfreebl3.so" ]; then sed -i "s_/etc/passwd_/not/anywhr_g" "/opt/google/earth/free/libfreebl3.so"; fi

# add non-root user for tests and production
ENV RUN_USER lg
ENV HOME /home/${RUN_USER}
RUN \
      useradd -ms /bin/bash $RUN_USER && \
      usermod -a -G sudo,plugdev,audio,video $RUN_USER && \
      mkdir -p $HOME/src ;\
      echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bash_profile ;\
      echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bash_profile ;\
      mv /bin/sh /bin/sh.bak && ln -s /bin/bash /bin/sh && \
      mkdir -p $PROJECT_ROOT/src

COPY ros_entrypoint.sh ${PROJECT_ROOT}

# clone appctl
# TODO change to latest tag
ARG APPCTL_TAG=python3_change
RUN git clone --branch ${APPCTL_TAG} https://github.com/EndPointCorp/appctl.git /appctl
RUN ln -snf /appctl/appctl ${PROJECT_ROOT}/

# pre-install dependencies for each package
COPY interactivespaces_msgs/package.xml ${PROJECT_ROOT}/interactivespaces_msgs/package.xml
COPY lg_activity/package.xml ${PROJECT_ROOT}/lg_activity/package.xml
COPY lg_attract_loop/package.xml ${PROJECT_ROOT}/lg_attract_loop/package.xml
COPY lg_builder/package.xml ${PROJECT_ROOT}/lg_builder/package.xml
COPY lg_common/package.xml ${PROJECT_ROOT}/lg_common/package.xml
COPY lg_earth/package.xml ${PROJECT_ROOT}/lg_earth/package.xml
COPY lg_json_config/package.xml ${PROJECT_ROOT}/lg_json_config/package.xml
COPY lg_keyboard/package.xml ${PROJECT_ROOT}/lg_keyboard/package.xml
COPY lg_media/package.xml ${PROJECT_ROOT}/lg_media/package.xml
COPY lg_mirror/package.xml ${PROJECT_ROOT}/lg_mirror/package.xml
COPY lg_nav_to_device/package.xml ${PROJECT_ROOT}/lg_nav_to_device/package.xml
COPY lg_lock/package.xml ${PROJECT_ROOT}/lg_lock/package.xml
COPY lg_offliner/package.xml ${PROJECT_ROOT}/lg_offliner/package.xml
COPY lg_panovideo/package.xml ${PROJECT_ROOT}/lg_panovideo/package.xml
COPY lg_pointer/package.xml ${PROJECT_ROOT}/lg_pointer/package.xml
COPY lg_proximity/package.xml ${PROJECT_ROOT}/lg_proximity/package.xml
COPY lg_replay/package.xml ${PROJECT_ROOT}/lg_replay/package.xml
COPY lg_rfreceiver/package.xml ${PROJECT_ROOT}/lg_rfreceiver/package.xml
COPY lg_screenshot/package.xml ${PROJECT_ROOT}/lg_screenshot/package.xml
COPY lg_spacenav_globe/package.xml ${PROJECT_ROOT}/lg_spacenav_globe/package.xml
COPY lg_stats/package.xml ${PROJECT_ROOT}/lg_stats/package.xml
COPY lg_sv/package.xml ${PROJECT_ROOT}/lg_sv/package.xml
COPY lg_twister/package.xml ${PROJECT_ROOT}/lg_twister/package.xml
COPY lg_volume_control/package.xml ${PROJECT_ROOT}/lg_volume_control/package.xml
COPY lg_wireless_devices/package.xml ${PROJECT_ROOT}/lg_wireless_devices/package.xml
COPY liquidgalaxy/package.xml ${PROJECT_ROOT}/liquidgalaxy/package.xml
COPY rfid_scanner/package.xml ${PROJECT_ROOT}/rfid_scanner/package.xml
COPY rfreceiver/package.xml ${PROJECT_ROOT}/rfreceiver/package.xml
COPY spacenav_remote/package.xml ${PROJECT_ROOT}/spacenav_remote/package.xml
COPY spacenav_wrapper/package.xml ${PROJECT_ROOT}/spacenav_wrapper/package.xml
COPY state_proxy/package.xml ${PROJECT_ROOT}/state_proxy/package.xml
COPY wiimote/package.xml ${PROJECT_ROOT}/wiimote/package.xml
RUN \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt-get update && \
    rosdep init && \
    rosdep update --include-eol-distros && \
    rosdep install \
        --from-paths $PROJECT_ROOT \
        --ignore-src \
        --rosdistro $ROS_DISTRO \
        -y && \
    rm -rf /var/lib/apt/lists/*

# install the full package contents
COPY ./ ${PROJECT_ROOT}
RUN \
    cd ${PROJECT_ROOT} && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    /ros_entrypoint.sh ./scripts/init_workspace && \
    cd ${PROJECT_ROOT}/catkin/ && \
    catkin_make && \
    catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO install && \
    source $PROJECT_ROOT/catkin/devel/setup.bash && \
    chown -R ${RUN_USER}:${RUN_USER} ${PROJECT_ROOT} && \
    chown -R ${RUN_USER}:${RUN_USER} ${HOME}

USER $RUN_USER

# by default let's run tests
#CMD cd ${PROJECT_ROOT}/catkin && \
#    . devel/setup.sh && \
#    cd ${PROJECT_ROOT} && \
#    ./scripts/docker_xvfb_add.sh && \
#    ./scripts/test_runner.py
