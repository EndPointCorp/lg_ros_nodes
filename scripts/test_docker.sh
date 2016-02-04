cd $(dirname $0)
DIR=`pwd -P`
DOCKER_NAME=

function setup_files() {
  # make a destination for the files
	mkdir -p docker_nodes/
	# copy all nodes (following links) into the proper directory
	cp -rpH catkin/src/* docker_nodes/
}

function initialize() {
  # change to the project root directory
	cd ${DIR}/..
	DOCKER_NAME="$(basename $(pwd))-test"
}

# builds the docker image, naming it <project-name>-test depending
# on the dirname of the project, e.g. lg_ros_nodes-test
function build_docker() {
  echo building ${DOCKER_NAME}
	docker build -t ${DOCKER_NAME} .
}

# runs tests and returns the return value
function run_tests() {
  echo running ${DOCKER_NAME}
	docker run -it -v $(pwd)/docker_nodes:/docker_nodes:ro ${DOCKER_NAME}
}

initialize
setup_files
build_docker
run_tests
