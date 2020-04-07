pipeline {
  agent { 
    dockerfile {
      args "-u 0 --env='DISPLAY=:1' -v /home/galadmin/inc:/home/lg/inc"
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "cd /src/lg_ros_nodes/catkin && \
	    . devel/setup.sh && \
	    cd /src/lg_ros_nodes && \
	    ./scripts/docker_xvfb_add.sh && \
	    ./scripts/test_runner.py \
	    "
      }
    }

  }
}
