pipeline {
    agent any

    environment {
        SONAR_TOKEN = credentials('jenkins_token')  // Jenkins credential ID
        PIO_CORE_DIR = "${WORKSPACE}/Code/Esp32/.platformio"
        VENV_PATH = "${WORKSPACE}/pio-venv"
        GITHUB_TOKEN = credentials('toan') // Add this in Jenkins credentials
        REPO_OWNER = 'nguyenngoctoancnh' // Replace with your GitHub username
        REPO_NAME = '21IO2' // Replace with your repository name
    }

    stages {
        stage('Checkout Code') {
            steps {
                checkout scm
            }
        }
         stage('SonarQube Analysis') {
            steps {
                withSonarQubeEnv('admin') {
                    sh '/opt/sonar-scanner/bin/sonar-scanner -X -Dsonar.projectKey=21IO2 -Dsonar.sources=./Code -Dsonar.login=sqa_8c0e216adf98f98627024e9b362bafaac773cf2a'
                }
            }
        }
        stage('Install PlatformIO') {
            steps {
                sh '''
                    rm -rf "$VENV_PATH"
                    python3 -m venv "$VENV_PATH"
                    bash -c "source $VENV_PATH/bin/activate && pip install --upgrade pip && pip install platformio"
                    bash -c "source $VENV_PATH/bin/activate && pio --version"
                '''
            }
        }

        stage('Build') {
            steps {
                dir('Code/Esp32') {
                    sh '''
                        echo "Disk space:"
                        df -h
                        echo "Memory before build:"
                        free -m
                        bash -c "source $VENV_PATH/bin/activate && pio run -v"
                    '''
                }
            }
        }
    }

    post {
        success {
            echo '✅ Build succeeded!'
            
            // // Archive artifacts in Jenkins
            // archiveArtifacts artifacts: 'Code/Esp32/.pio/build/featheresp32/*.hex,Code/Esp32/.pio/build/featheresp32/*.elf', allowEmptyArchive: true
            
            // Create a GitHub release and upload artifacts
            sh '''
                # Create a release with a tag based on the build number
                TAG="build-${BUILD_NUMBER}"
                curl -X POST \
                    -H "Authorization: token $GITHUB_TOKEN" \
                    -H "Accept: application/vnd.github.v3+json" \
                    https://api.github.com/repos/$REPO_OWNER/$REPO_NAME/releases \
                    -d "{\"tag_name\":\"$TAG\", \"name\":\"Build #$BUILD_NUMBER\", \"body\":\"Artifacts from Jenkins build #$BUILD_NUMBER\", \"draft\":false, \"prerelease\":false}" > release_response.json
                
                # Extract the release ID
                RELEASE_ID=$(cat release_response.json | jq -r '.id')
                
                # Upload .hex file
                if [ -f "Code/Esp32/.pio/build/featheresp32/firmware.hex" ]; then
                    curl -X POST \
                        -H "Authorization: token $GITHUB_TOKEN" \
                        -H "Accept: application/vnd.github.v3+json" \
                        -H "Content-Type: application/octet-stream" \
                        --data-binary @Code/Esp32/.pio/build/featheresp32/firmware.hex \
                        https://uploads.github.com/repos/$REPO_OWNER/$REPO_NAME/releases/$RELEASE_ID/assets?name=firmware.hex
                fi
                
                # Upload .elf file
                if [ -f "Code/Esp32/.pio/build/featheresp32/firmware.elf" ]; then
                    curl -X POST \
                        -H "Authorization: token $GITHUB_TOKEN" \
                        -H "Accept: application/vnd.github.v3+json" \
                        -H "Content-Type: application/octet-stream" \
                        --data-binary @Code/Esp32/.pio/build/featheresp32/firmware.elf \
                        https://uploads.github.com/repos/$REPO_OWNER/$REPO_NAME/releases/$RELEASE_ID/assets?name=firmware.elf
                fi
                
                # Clean up
                rm -f release_response.json
            '''
        }
        failure {
            echo '❌ Build failed.'
            // githubNotify context: 'Jenkins CI', status: 'FAILURE', description: 'Build failed'
        }
        always {
            echo '🧹 Cleaning up workspace...'
        }
    }
}
