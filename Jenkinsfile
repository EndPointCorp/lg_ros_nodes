pipeline {
  agent {
    dockerfile {
      filename 'Dockerfile'
    }

  }
  stages {
    stage('Setup') {
      steps {
        sh './scripts/run_ros_setup.sh'
        sh './scripts/test_docker.sh'
      }
    }

  }
}