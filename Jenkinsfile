pipeline {
  agent { 
    dockerfile {
      args "-u 0 --env='DISPLAY=:1' -v /home/galadmin/inc:/home/lg/inc"
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "./scripts/run_tests.sh"
      }
    }

  }
}
