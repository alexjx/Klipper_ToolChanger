#!/bin/bash
# Constants
EXTENSION_NAME="klipper_toolchanger"

# Force script to exit if an error occurs
set -e

# Find SRCDIR from the pathname of this script
SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/ && pwd )"

KLIPPER_PATH="${HOME}/klipper"
# Parse command line arguments to allow KLIPPER_PATH override
while getopts "k:" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
    esac
done

# Verify conditions for the install to take place
check_preconditions()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi

    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "klipper.service")" ]; then
        echo "Klipper service found!"
    else
        echo "Klipper service not found, please install Klipper first"
        exit -1
    fi
}

# Step 2: create a symlinks to the extension files in the klippy/extras directory
link_extension()
{
    echo "Linking ${EXTENSION_NAME} to Klippy extras..."
    for module in alignment.py tool.py toolgroup.py toollock.py ktcclog.py; do
        ln -sf "${SRCDIR}/${module}" "${KLIPPER_PATH}/klippy/extras/${module}"
    done
    package_target="${KLIPPER_PATH}/klippy/extras/ktcc"
    if [ -e "${package_target}" ] && [ ! -L "${package_target}" ]; then
        echo "Refusing to replace non-symlink path: ${package_target}"
        exit 1
    fi
    ln -sfn "${SRCDIR}/ktcc" "${package_target}"
}

# Step 3: restarting Klipper
restart_klipper()
{
    echo "Restarting Klipper..."
    sudo systemctl restart klipper
}

# Installation steps:
check_preconditions
link_extension
restart_klipper
