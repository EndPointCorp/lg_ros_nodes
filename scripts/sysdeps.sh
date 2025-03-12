#!/usr/bin/env bash
# Write each package's system deps to a separate file,
#   so that ROS is not required to install the deps.

REPO_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )
ROS_DISTRO=${ROS_DISTRO:-noetic}

cd ${REPO_DIR}
SUBDIRS=$(find . -type f -name "package.xml" -exec dirname "{}" \;)

apt-get update
. /opt/ros/${ROS_DISTRO}/setup.sh
rosdep update --rosdistro ${ROS_DISTRO}
for subdir in ${SUBDIRS[*]}; do
  for key in $(rosdep keys -y --from-paths ${subdir} --rosdistro ${ROS_DISTRO} --ignore-src \
    --skip-keys=python-rospkg \
    --skip-keys=python-requests \
    --skip-keys=python-serial \
    --skip-keys=python-catkin-pkg \
    --skip-keys=python-debian \
    --skip-keys=python-evdev \
    --skip-keys=python-cwiid \
    --skip-keys=python-oauth2client \
    --skip-keys=python-gst-1.0); do \
    rosdep resolve ${key} | grep -v '^#' >> "${subdir}/sys.deps"
  done
done

