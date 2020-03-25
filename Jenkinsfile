pipeline {
  agent { 
    dockerfile {
      args "-u 0 --rm --env='DISPLAY=:0' -v /home/galadmin/inc:/home/lg/inc"
    }
  }
  stages {
    stage('Test') {
      steps {
        input('pause to try things')
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
