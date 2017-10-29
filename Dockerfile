# Basic docker file for running tests, lives in the root directory because
# it needs to add the entire project into the container
FROM endpoint/lg_ros_nodes_base:0.1
MAINTAINER Jacob Minshall <jacob@endpoint.com>

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
    cd ${PROJECT_ROOT}/catkin/ && \
    apt-get update && \
    apt-get install --yes python-pip python-debian && \
    pip install python-coveralls && \
    rosdep init &&\
    rosdep update && \
    sudo rosdep install \
        --from-paths /home/galadmin/src/lg_ros_nodes/catkin/src \
        --ignore-src \
        --rosdistro indigo \
        -y && \
    catkin_make && \
    catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo install && \
    source /home/galadmin/src/lg_ros_nodes/catkin/devel/setup.bash && \
    sudo chown -R ${TEST_USER}:${TEST_USER} ${HOME} && \
    rm -rf /var/lib/apt/lists/*

# by default let's run tests
CMD cd ${PROJECT_ROOT}/catkin && \
    . devel/setup.sh && \
    cd ${PROJECT_ROOT} && \
    ./scripts/docker_xvfb_add.sh && \
    ./scripts/test_runner.py
