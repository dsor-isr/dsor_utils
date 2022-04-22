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
        PACKAGE_NAME = 'dsor_utils'
    }
    stages {
        stage('Setup') {
            sh 'printenv'
            sh '''
                mkdir -p ${ROS_WORKSPACE}/src
                cp -R . ${ROS_WORKSPACE}/src'''
            sh '''
                cd ${ROS_WORKSPACE}/src
                ls'''
        }
        // Build stage - compile the code
        stage('Build') {
            steps {
                echo 'Build..'
                dir('${ROS_WORKSPACE}') {
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
