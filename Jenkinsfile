pipeline {
  agent { 
    dockerfile {
      args "-u 0"
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "./scripts/run_tests.sh"
      }
    }
    stage('Build') {
      when {
        branch 'master'
      }
      steps {
        sh "./scripts/build.sh"
      }
    }
    stage('Deploy') {
      when {
        branch 'master'
      }
      steps {
        sh "./scripts/deploy.sh"
      }
    }
  }
}
