// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

pipeline {
    agent {
        docker {
            image 'wpilib/roborio-cross-ubuntu:2025-22.04'
        }
    }
    environment {
        GIT_SAFE_DIR = "${WORKSPACE}"
    }
    stages {
        stage('Checkout') {
            steps {
                checkout scm
                sh 'git config --global --add safe.directory ${GIT_SAFE_DIR}'
            }
        }
        stage('Grant Permissions') {
            steps {
                sh 'chmod +x gradlew'
            }
        }
        stage('Build and Test') {
            steps {
                sh './gradlew build'
            }
        }
    }
}
