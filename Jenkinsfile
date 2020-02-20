pipeline {
  agent {
    dockerfile {
      filename 'Dockerfile'
    }

  }
  stages {
    stage('Setup') {
      steps {
        sh './scripts/test_docker.sh'
      }
    }

  }
}