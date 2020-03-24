pipeline {
  agent { 
    dockerfile {
      args "-u 0 --rm --env='DISPLAY=:0'"
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "pwd && \
	    cd /src/lg_ros_nodes/catkin && \
	    . devel/setup.sh && \
	    cd /src/lg_ros_nodes && \
	    ./scripts/docker_xvfb_add.sh && \
	    ./scripts/test_runner.py \
	    "
      }
    }

  }
}
