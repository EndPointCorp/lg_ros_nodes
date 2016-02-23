# Liquid Galaxy
#
# VERSION 0.1

FROM 	ubuntu:14.04.3

# Install basic stuff
RUN     apt-get install -y wget curl tmux git

# Add deb repos
RUN 	echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list ;\
      apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 5523BAEEB01FA116 ;\
      wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add - ;\
      echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list ;\
      wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add - ;\
      echo "deb http://dl.google.com/linux/earth/deb/ stable main" > /etc/apt/sources.list.d/google.list


# Stuff for GE and Chrome
RUN     apt-get update && apt-get install -y \
            x-window-system \
            binutils \
            mesa-utils \
            mesa-utils-extra \
            module-init-tools \
            gdebi-core \
            tar \
            libfreeimage3 ;\
        apt-get install -y --no-install-recommends xdg-utils

# Nvidia drivers
ADD 	nvidia-driver.run /tmp/nvidia-driver.run
RUN 	sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module ;\
      rm /tmp/nvidia-driver.run

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

# Install ROS
RUN     apt-get update && apt-get install -y \
            ros-indigo-ros-base \
            lsb-core \
            google-chrome-stable

# Env
RUN     rm /bin/sh && ln -s /bin/bash /bin/sh
RUN	    mkdir /home/galadmin && mkdir /home/galadmin/src ;\
        echo "source /opt/ros/indigo/setup.bash" >> /root/.bashrc

WORKDIR /home/galadmin/src
RUN	    git clone git://github.com/EndPointCorp/lg_ros_nodes.git ;\
	      git clone git://github.com/EndPointCorp/appctl.git

RUN     apt-get install -y ros-indigo-rosbridge-server

WORKDIR /home/galadmin/src/lg_ros_nodes
RUN     source /opt/ros/indigo/setup.bash ;\
        rosdep init ;\
        rosdep update ;\
        ./scripts/init_workspace --appctl /home/galadmin/src/appctl ;\
        cd /home/galadmin/src/lg_ros_nodes/catkin ;\
        rosdep install --from-paths src --ignore-src --rosdistro indigo -y ;\
        catkin_make ;\
        source /home/galadmin/src/lg_ros_nodes/catkin/devel/setup.bash ;\
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo install

WORKDIR /home/galadmin/src/lg_ros_nodes
ENV     DISPLAY :0

ADD     image_scripts/shell.sh /home/galadmin/shell.sh
RUN     chmod +x /home/galadmin/shell.sh

CMD     /bin/bash -c /home/galadmin/shell.sh
