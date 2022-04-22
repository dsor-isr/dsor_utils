pipeline {
    agent {
        docker {
            image 'harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:v0.0.3'
            registryUrl 'https://harbor.dsor.isr.tecnico.ulisboa.pt'
            registryCredentialsId 'harbor-robot-token'
            args '--entrypoint="" -v ${HOME}/catkin_ws/src:/var/jenkins_home/workspace/Hub_DSOR_dsor_utils_dev_pipeline/catkin_ws/src'
        }
    }
    stages {
        // Build stage - compile the code
        stage('Build') {
            steps {
                echo 'Build..'
                dir('catkin_ws/src') {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    catkin build --no-status
                '''
                }
            }
        }
        // Test stage - test the code
        stage('Test') {
            steps {
                echo 'Testing..'
            }
        }
        // Generate Doxygen documentation
        stage('Documentation') {
            steps{
                echo 'Generating Doxygen Documentation..'
            }
        }
    }
}
