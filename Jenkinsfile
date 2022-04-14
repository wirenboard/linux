pipeline {
    agent {
        label 'devenv'
    }
    parameters {
        booleanParam(name: 'ADD_VERSION_SUFFIX', defaultValue: true, description: 'for non dev/* branches')
        booleanParam(name: 'UPLOAD_TO_POOL', defaultValue: true,
                     description: 'works only with ADD_VERSION_SUFFIX to keep staging clean')
        booleanParam(name: 'CLEAN', defaultValue: false, description: 'force cleaned on dev/* branches')
        booleanParam(name: 'FORCE_OVERWRITE', defaultValue: false, description: 'replace existing version of package in apt')
        booleanParam(name: 'REPLACE_RELEASE', defaultValue: false, description: 'replace existing github release')
        string(name: 'KERNEL_FLAVOUR', defaultValue: 'wb2 wb6 wb7', description: 'space-separated list')
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
                        [$class: 'CloneOption', timeout: 30, shallow: true, depth: 300,
                         reference: '/home/jenkins/userContent/linux-reference'],
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
            when { anyOf {
                expression {
                    params.CLEAN
                }
                branch 'dev/*'
            }}
            steps {
                sh """wbdev user make mrproper && \\
                      for f in $KERNEL_FLAVOUR; do rm .build-\$f -rf; done"""
            }
        }
        stage('Determine version suffix') {
            when { expression {
                params.ADD_VERSION_SUFFIX && !wb.isBranchRelease(env.BRANCH_NAME, 'dev/(.*)')
            }}

            steps {
                script {
                    def baseCommit = sh(returnStdout: true, script: '''\\
                        git log --diff-filter=A --cherry --pretty=format:"%h" -- debian/changelog''').trim()

                    def versionSuffix = sh(returnStdout: true, script: """\\
                        echo ~exp~`echo ${BRANCH_NAME} | sed -e 's/\\W/+/g' -e 's/_/+/g'`~`\\
                        git rev-list --count HEAD...${baseCommit}`~g`\\
                        git rev-parse --short HEAD`""").trim()
                    env.VERSION_SUFFIX = versionSuffix
                }
            }
        }
        stage('Setup builds') {
            steps {
                script {
                    def flavours = params.KERNEL_FLAVOUR.split(' ')

                    for (flavour in flavours) {
                        def currentFlavour = flavour  // to refer in closure
                        stage("Build ${currentFlavour}") {
                            sh """wbdev user \\
                               KERNEL_FLAVOUR=${currentFlavour} \\
                               VERSION_SUFFIX=\$VERSION_SUFFIX \\
                               FORCE_DEFAULT=n \\
                               scripts/package/wb/do_build_deb.sh"""
                        }
                    }
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

        stage('Setup deploy') {
            when { expression {
                params.UPLOAD_TO_POOL && params.ADD_VERSION_SUFFIX
            }}
            steps {
                wbDeploy projectSubdir: '.',
                         resultSubdir: env.RESULT_SUBDIR,
                         forceOverwrite: params.FORCE_OVERWRITE,
                         replaceRelease: params.REPLACE_RELEASE,
                         customReleaseBranchPattern: 'dev/(.*)'
            }
        }
    }
}
