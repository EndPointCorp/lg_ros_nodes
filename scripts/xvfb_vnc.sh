# This script is useful when you want to run
# tests using headless Xvfb and connect via
# vnc to it to debug chrome browsers
echo "Starting Xvfb display :99"
Xvfb :99 -ac $randr $xinerama -nolock \
      -nocursor -screen 0 1920x1080x24 \
          &> /tmp/xvfb_start.log &
echo "Starting X11vnc to export display :99 on port 5900"
x11vnc -localhost -display :99
echo "Now connect to port 5900 on localhost"
echo "You can make a forward like ssh <hostname> -L 5900:localhost:5900"
