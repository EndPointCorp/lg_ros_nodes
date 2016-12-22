# lg_ros_nodes_base squashed image

This is the docker image that's located under
https://hub.docker.com/r/endpoint/lg_ros_nodes_base/

## Requirements

- docker toolbox
- docker-squash (pip install docker-squash)

## Creating the image

- create the new untagged docker image (in this directory:
NOTE: execute this command withing this directory

```bash
docker build -f Dockerfile ../
```

then:

```bash
docker images
```

to get you newly built docker image ID

- remove all cruft from the docker image after building

```bash
docker run -it <docker image id> /bin/bash
apt-get autoclean clean
apt-get autoremove
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

```bash
docker-squash <new image id> -t endpoint/lg_ros_nodes_base:<new version tag>
```

- push your docker image

```bash
docker login
docker push endpoint/lg_ros_nodes_base
```

