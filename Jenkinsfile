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
        sh "./pack-debs master"
      }
    }
  }
}

