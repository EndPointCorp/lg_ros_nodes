pipeline {
  agent { 
    dockerfile {
      args "-u 0 --rm --env='DISPLAY=:0'"
    }
  }
  stages {
    stage('Test') {
      steps {
        input('Pause until procede')
        sh "pwd && \
	    cd /home/lg/src/lg_ros_nodes/catkin && \
	    . devel/setup.sh && \
	    cd /home/lg/src/lg_ros_nodes && \
	    ./scripts/docker_xvfb_add.sh && \
	    ./scripts/test_runner.py \
	    "
      }
    }

  }
}
