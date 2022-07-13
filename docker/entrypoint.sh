#!/bin/bash
SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
SCRIPT_NAME="$(basename "$0")"

SUPPORTED_UBUNTU_VERSION_ID="20.04"

SRC_DIR="${SRC_DIR:-${HOME}/src}"
BUILD_DIR="${BUILD_DIR:-${HOME}}"

CHIP_DIR="connectedhomeip"
CHIP_GIT="https://github.com/project-chip/connectedhomeip.git"
CHIP_COMMIT="master"

ESPIDF_DIR="esp-idf"
ESPIDF_GIT="https://github.com/espressif/esp-idf.git"
ESPIDF_COMMIT="v4.4.1"
ESPIDF_TARGET="esp32c3"

ESPRESSIF_DIR="espressif"

PROJECT_NAME="all-clusters-app"

die() { echo "$*" 1>&2; exit 1; }
exit_signal() { rc=$?; echo "$rc $*"; exit $rc; }
trap 'exit_signal SIGINT' INT
trap 'exit_signal EXIT' EXIT

function init_container_user() {
	if [ -z ${USER_ID} ]; then
		# Default values if user info not provided
		export USER_NAME=builder
		export USER_ID=1000
		export GROUP_ID=1000
		export HOME=/home/${USER_NAME}
	fi

	if ! id "${USER_NAME}" >/dev/null 2>&1; then
		# This assumes that group name is the same as user name for now
		groupadd -o --gid ${GROUP_ID} ${USER_NAME}
		# Create non-root user
		useradd -o --uid ${USER_ID} --gid ${GROUP_ID} --groups ${TTY_GROUP_ID} --home-dir ${HOME} ${USER_NAME}
		# setup sudo without password for user
		echo "${USER_NAME} ALL=NOPASSWD: ALL" > /etc/sudoers.d/user-nopasswd
		chmod 660 /etc/sudoers.d/user-nopasswd
	fi

	if [ ! -d ${HOME} ]; then
		mkdir -p ${HOME}
		chown ${USER_NAME}:${USER_NAME} ${HOME}
	fi
}

#######
# Start
CURRENT_USER="$(whoami)"
#echo "Running docker entrypoint as \"${CURRENT_USER}\"..."
if [ -r /etc/os-release ] && [ "${CURRENT_USER}" = "root" ]; then
	source /etc/os-release
	if [[ "${VERSION_ID}" == "${SUPPORTED_UBUNTU_VERSION_ID}" ]] && [[ "${USER_NAME}" == "builder" ]]; then
		init_container_user
	fi
	gosu ${USER_NAME} bash -c "$*"
fi
