#!/usr/bin/env groovy
pipeline {

    agent { dockerfile true }

    environment {
        GIT_SHORT_SHA = (env.GIT_COMMIT).substring(0,7)
        FIRMWARE_FILE = "ArduCopter-3.5.4-ireq-${GIT_SHORT_SHA}.px4"
    }

    stages {

        stage('Update submodules') {
            steps {
                // Recursively update git submodules
                sh '''
                    git submodule update --init --recursive
                '''
            }
        }

        stage('Build sitl') {
            steps {
                sh '''
                    ./waf distclean
                    rm -f build/px4-v2/bin/*.px4
                    ./waf configure --board=sitl
                    ./waf copter
                '''
            }
        }

        stage('Run tests') {
            steps {
                sh '''
                    ./waf check
                '''
            }
        }

        stage('Build px4-v2') {
            steps {
                sh '''
                    ./waf distclean
                    ./waf configure --board=px4-v2
                    ./waf copter
                '''
            }
        }

        stage('Archive px4-v2 firmware') {
            steps {
                sh '''
                    cp build/px4-v2/bin/arducopter.px4 build/px4-v2/bin/${FIRMWARE_FILE}
                '''
                archiveArtifacts artifacts: 'build/px4-v2/bin/*ireq*.px4'
            }
        }
    }
}

