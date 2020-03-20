pipeline {
  agent { 
    dockerfile {
      args "-u 0 --rm --env='DISPLAY=:0'"
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "cd ${WORKSPACE}/catkin && \
	    . devel/setup.sh && \
	    cd ${WORKSPACE} && \
	    ./scripts/docker_xvfb_add.sh && \
	    ./scripts/test_runner.py \
	    "
      }
    }

  }
}
