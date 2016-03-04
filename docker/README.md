# Run Liquid galaxy inside Docker container

This docker image is a quick start for Liquid Galaxy.

It allows you to test the idea of how LG works on your desktop without
installation of additional hardware.

## Build

Run `./build.sh` or do it manually with `sudo docker build -t lgros ${DOCKER_DIR}`

## Run

`./run.sh` runs the container.

## Inside the container

There are three predefined launch configurations:
- `earth.launch` or just execute `launch-earth` Starts three synchronized instances of Google Earth.

  The full run command is: `roslaunch --screen lg_common/launch/earth.launch broadcast_addr:=224.0.0.1`

- `sv.launch` or just execute `launch-sv` Starts three Google Chrome browsers with Street View.

  To set the actual view, open new tmux window (Ctrl-B c) then run for a second.
  `rostopic pub /streetview/panoid std_msgs/String "data: 'LsT9Nj27lXssBJW0VgoIPA'"`

  It will cause the Street View to pic the actual panorama. To get panorama id
  see: https://developers.google.com/maps/documentation/javascript/streetview

- `pv.launch` or just execute `launch-pv` Starts panorama view.  
