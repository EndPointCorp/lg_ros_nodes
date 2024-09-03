pipeline {
  environment {
    APTLY_SERVER = credentials('aptly-server-url')
  }
  agent { 
    dockerfile {
      args "-u 0 -v /var/lib/jenkins/.ssh/:/root/ssh"
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
