pipeline {
  agent none
  stages {
    stage('Setup') {
      agent any
      steps {
        sh ./scripts/setup_tests.sh
      }
    }
    stage('Test') {
      agent { 
        dockerfile {
	  args "-u 0 --rm --env='DISPLAY=:0'"
	}
      }
      steps {
        sh 'cd ${PROJECT_ROOT}/catkin && \
	    . devel/setup.sh && \
	    cd ${PROJECT_ROOT} && \
	    ./scripts/docker_xvfb_add.sh && \
	    ./scripts/test_runner \
	    '
      }
    }

  }
}
