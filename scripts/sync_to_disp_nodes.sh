#!/bin/bash
# Script used to syncronize catkin workspace between headnode and dispnodes

if [ -d .git ]; then
  echo "I'm going to sync catkin/src to dispnodes:catkin_ws/src/ and run catkin_make"
  echo "press enter to go"
  read
else
  echo "Not in a git repository - exiting"
  exit 1
fi

if [ -d catkin/src ] ; then
  echo "..."
else
  echo "There's no catkin/src - have you initialized your workspace?"
  exit 1
fi

flag=false
for d in `ls catkin/src/`; do
  loc="$(readlink -f catkin/src/${d})"
  test -d ${loc} || continue  # ignoring non directories
  if [[ ${loc} =~ ${PWD} ]]; then
    continue
  fi
  if [ -d ${loc}/.git ] && [ -d ${loc}/../.git ]; then
    echo "Non standard repo (.git isn't in current or ../ dir): ${loc}"
    flag=true
    continue
  fi
  cd "${loc}"
  git remote update 1>/dev/null
  if [ "$?" != "0" ]; then
    flag=true
  fi
  dff=$(git diff origin/master | wc -l)
  if [ "${dff}" != "0" ]; then
    echo "WARNING: ${loc} is not pointing to the latest master..."
    flag=true
  fi
  cd - 2>/dev/null 1>/dev/null
done
if [ "${flag}" != "false" ]; then
  echo "there were non-updated repos in your catkin/src"
  read -rp "would you like to continue? (y/n) " abort
  if [ "${abort}" != "Yes" ] && [ "${abort}" != "y" ] && [ "${abort}" != "Y" ] && [ "${abort}" != "yes" ]; then
    echo "Goodbye!"
    exit 0
  fi
fi

echo 'installing libudev-dev'
if [[ ${SKIP_APT} != "" ]]; then
  echo "Skipping APT update + install"
else
  lg-sudo-bg -w "sudo apt-get update -q && sudo apt-get install build-essential gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good python-gst-1.0 libudev-dev gstreamer1.0-x ros-${ROS_DISTRO}-web-video-server -q -y"
  echo hi
fi

lg-sync --really-sync catkin/src/ /home/lg/catkin_ws/src/
lg-run-bg -w "cd catkin_ws ; \
           source /opt/ros/${ROS_DISTRO}/setup.bash ; \
           if [ ! -e /tmp/rosdep_updated ]; then
             echo "updating rosdep, please wait"
             rosdep update;
             touch /tmp/rosdep_updated;
           fi
           rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y;
           catkin_make clean;\
           catkin_make install;\
           find /home/lg/catkin_ws/src/ -iname '*.pyc' -delete "

# Matt changes
lg-sudo-bg -w "cd /home/lg/catkin_ws ; \
        source /opt/ros/${ROS_DISTRO}/setup.bash ; \
        catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} "


echo 'linking extensions to /opt/google/chrome/extensions/'
lg-sudo-bg -w 'sudo mkdir -p /opt/google/chrome/extensions/; sudo ln -sf /home/lg/catkin_ws/src/lg_common/src/lg_common/extensions/* /opt/google/chrome/extensions/'

echo "You need to relaunch to new version"
