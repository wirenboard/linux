pipeline {
    agent {
        label 'devenv'
    }
    parameters {
        booleanParam(name: 'ADD_VERSION_SUFFIX', defaultValue: true, description: 'for non dev/* branches')
        booleanParam(name: 'UPLOAD_TO_POOL', defaultValue: true, description: 'for dev/* packages')
        booleanParam(name: 'CLEAN', defaultValue: true, description: 'clean tree before build')
        string(name: 'KERNEL_FLAVOUR', defaultValue: 'wb2 wb6', description: 'space-separated list')
        string(name: 'WBDEV_IMAGE', defaultValue: '', description: 'docker image path and tag')
    }
    environment {
        WBDEV_BUILD_METHOD = 'sbuild'
        WBDEV_TARGET = 'wb6'
        RESULT_SUBDIR = 'pkgs'
    }
    options {
        skipDefaultCheckout()
    }
    stages {
        stage('Checkout with submodules') {
            // TODO: common checkout to a local storage can save time if you are building different branches
            steps {
                checkout scm: [
                    $class: 'GitSCM',
                    branches: scm.branches,
                    extensions: [
                        [$class: 'CloneOption', timeout: 30 ],
                        [$class: 'SubmoduleOption', disableSubmodules: false, recursiveSubmodules: true]
                    ],
                    userRemoteConfigs: scm.userRemoteConfigs
                ]
            }
        }
        stage('Cleanup workspace') {
            steps {
                cleanWs deleteDirs: true, patterns: [[pattern: "$RESULT_SUBDIR", type: 'INCLUDE']]
            }
        }
        stage('Clean build tree') {
            when { expression {
                params.CLEAN
            }}
            steps {
                sh """wbdev user make mrproper && \\
                      for f in $KERNEL_FLAVOUR; do rm .build-\$f -rf; done"""
            }
        }
        stage('Determine version suffix') {
            when {
                not {
                    branch 'dev/*'
                }
                expression {
                    params.ADD_VERSION_SUFFIX
                }
            }

            steps {
                script {
                    def versionSuffix = sh(returnStdout: true, script:'''\\
                        . ./wb_revision; \\
                        echo ~`echo ${BRANCH_NAME} | sed 's/[/~^ \\_\\-]/+/g'`+`\\
                        git rev-list --count HEAD...${WB_BRANCH_BASE}`+g`\\
                        git rev-parse --short HEAD`''').trim()
                    env.WB_REVISION = versionSuffix
                }
            }
        }
        stage('Build packages') {
            steps {
                script {
                    def flavours = env.KERNEL_FLAVOUR.split(' ')
                    def jobs = [:]

                    for (flavour in flavours) {
                        def currentFlavour = flavour  // to refer in closure
                        jobs["build ${currentFlavour}"] = {
                            stage("Build ${currentFlavour}") {
                                sh """wbdev user \\
                                   WB_REVISION=$WB_REVISION \\
                                   KERNEL_FLAVOUR=${currentFlavour} \\
                                   FORCE_DEFAULT=y \\
                                   scripts/package/wb/do_build_deb.sh"""
                            }
                        }
                    }

                    parallel jobs
                }
            }
            post {
                always {
                    sh "mkdir -p $RESULT_SUBDIR && mv *.deb *.changes $RESULT_SUBDIR/"
                }
                success {
                    archiveArtifacts artifacts: "$RESULT_SUBDIR/*.deb"
                }
            }
        }

        stage('Add packages to pool') {
            when {
                expression {
                    params.UPLOAD_TO_POOL
                }
                branch 'dev/*'
            }

            environment {
                APTLY_CONFIG = credentials('release-aptly-config')
            }

            steps {
                sh 'wbci-repo -c $APTLY_CONFIG add-debs -f -d "jenkins:$JOB_NAME.$BUILD_NUMBER" $RESULT_SUBDIR/*.deb'
            }
        }
    
        stage('Upload via wb-releases') {
            when {
                expression {
                    params.UPLOAD_TO_POOL
                }
                branch 'dev/*'
            }

            steps {
                build job: 'contactless/wb-releases/master', wait: true
            }
        }
    }
}
