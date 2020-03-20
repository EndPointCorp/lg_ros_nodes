pipeline {
  agent { 
    dockerfile {
      args "-u 0 --rm -v $SSH_AUTH_SOCK:/ssh-agent --env='DISPLAY=:0'"
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "cd ${PROJECT_ROOT}/catkin && \
	    . devel/setup.sh && \
	    cd ${PROJECT_ROOT} && \
	    ./scripts/docker_xvfb_add.sh && \
	    ./scripts/test_runner.py \
	    "
      }
    }

  }
}
