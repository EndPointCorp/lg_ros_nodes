# Run Liquid galaxy inside Docker container

I use docker with root permissions, so both `buld.sh` and `run.sh`
scripts will ask you for sudo password.

## Build

Run `./build.sh` or do it manualy with `sudo docker build -t lgros ${DOCKER_DIR}`

## Run

`./run.sh` runs the container with tmux session.

There are three predefined launch configurations:
- `earth.launch` Starts three synchronized instances of Google Earth.

  The full run command is: `roslaunch --screen lg_common/launch/earth.launch broadcast_addr:=224.0.0.1`

- `sv.launch` Starts three Google Chrome browsers with Street View.

  To set the actual view, open new tmux window (Ctrl-B c) then run for a second.
  `rostopic pub /streetview/panoid std_msgs/String "data: 'LsT9Nj27lXssBJW0VgoIPA'"`

  It will cause the Street View to pic the actual panorama. To get panorama id
  see: https://developers.google.com/maps/documentation/javascript/streetview

- `pv.launch` Starts panorama view.  

## Development

Nvidia drivers inside docker via https://github.com/thewtex/docker-opengl-nvidia
