# Run Liquid galaxy inside Docker container

I use docker with root permissions, so both `buld.sh` and `run.sh`
scripts will ask you for sudo password.

## Build

`./build.sh` it will also download and install nvidia drivers with a same
version as on the host machine inside docker for proper X forwarding.

## Run

`./run.sh` runs the tmux session with roslaunch.

- `earth.launch` Starts three synchronized instances of Google Earth
- `dev.launch` default, same as `earth.launch`
- `browser.launch`

## Development

Nvidia drivers inside docker via https://github.com/thewtex/docker-opengl-nvidia
