pipeline {
	agent { 
		dockerfile {
			args '-e HOME=$WORKSPACE'
			additionalBuildArgs '--build-arg USER_ID=$(id -u)'
		}
	}
	stages {
		stage('Build') {
			steps {
				sh '''
          #!/bin/bash -l
          mkdir build
          cd build
          cmake ..
          make -j4
				'''
          // sudo make install
			}
		}
		stage('Test') {
			steps {
				echo 'Testing...'
				echo 'Run unit tests here'
			}
		}
	}
	post {
		cleanup {
			cleanWs()
		}
	}
}

