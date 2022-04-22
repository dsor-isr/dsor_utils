pipeline {
    agent {
        docker {
            image 'harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:v0.0.3'
            registryUrl 'https://harbor.dsor.isr.tecnico.ulisboa.pt'
            registryCredentialsId 'harbor-robot-token'
            args '--entrypoint=""'
        }
    }
    environment {
        ROS_WORKSPACE = "${HOME}/catkin_ws"
    }
    // Move all the packages to the default catkin workspace
    stages {
        stage('Setup') {
            steps {
                sh '''
                    printenv
                    cp -R ${ROS_WORKSPACE}/src
                    rm -r ${ROS_WORKSPACE}/src/catkin_ws
                    rm -r ${ROS_WORKSPACE}/src/catkin_ws@tmp'''
            }
        }
        // Build stage - compile the code
        stage('Build') {
            steps {
                echo 'Build..'
                dir(path: "${ROS_WORKSPACE}") {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    catkin build --no-status'''
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
