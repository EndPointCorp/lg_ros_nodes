# lg_ros_nodes_base squashed image

This is the docker image that's located under
https://hub.docker.com/r/endpoint/lg_ros_nodes_base/

## Requirements

- docker toolbox
- docker-squash (pip install docker-squash)

## Creating the image

- create the new untagged docker image:

```bash
docker build -f Dockerfile
```
- tag it with `endpoint/lg_ros_nodes_base`

```bash
docker tag 7abd22dfd915 endpoint/lg_ros_nodes_base
```

where 7abd22dfd915 is the image id taken from `docker images`

- remove all cruft from the docker image after building

```bash
docker run -it endpoint/lg_ros_nodes_base:latest /bin/bash
apt-get autoclean clean
<other stuffz possibly>
```

- note down the latest layer from command prompt of the above command
  and commit the changes you've made as an additional layer

```bash
docker commit b6d0bb7eec3b
```

- above command will produce new layer ID - this is our top layer that
  we will use for squashing (usage is defined
  [here](https://github.com/jwilder/docker-squash#usage))

- export the unsquashed image

```bash
docker save <top layer id> > ~/tmp/image.tar
```

- squash the image

```bash
docker-squash -i ~/tmp/image.tar -o ~/tmp/squashed.tar
```

- load squashed image for pushing

```bash
cat squashed.tar | docker load
docker images
```

- d
