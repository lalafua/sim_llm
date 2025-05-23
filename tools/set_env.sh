#!/bin/bash

# set PYTHONPATH
VENV_PATH=""
SITE_PACKAGES=""

if [ -d "./.venv" ]; then
    VENV_PATH="./.venv"
elif [ -d "../.venv" ]; then
    VENV_PATH="../.venv"
fi

if [ -n "$VENV_PATH" ]; then
    SITE_PACKAGES=$(find "$VENV_PATH" -type d -name "site-packages" | head -n 1)
    if [ -n "$SITE_PACKAGES" ]; then
        SITE_PACKAGES=$(readlink -f "$SITE_PACKAGES")
        if echo ":$PYTHONPATH:" | grep -q ":$SITE_PACKAGES:"; then
            echo "[INFO] $SITE_PACKAGES is already in PYTHONPATH"
        else
            export PYTHONPATH="$SITE_PACKAGES:$PYTHONPATH"
            echo "[INFO] Add $SITE_PACKAGES to PYTHONPATH."
        fi
    else 
        echo "[WARNING] No site-packages in $VENV_PATH."
    fi
else 
    echo "[WARNING] No venv directory was found."
fi

# set ROS_DOMAIN_ID
if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=1
    echo "[INFO] ROS_DOAMIN_ID was not set, defaulting to 1."
fi

# get OS Info
OS_Info=$(uname -a | head -n 1)

echo "============================================="
echo "OS Info: $OS_Info" 
echo "ROS_DISTRO: $ROS_DISTRO"
echo "PYTHONPATH: $PYTHONPATH"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "DEEPSEEK_API_KEY: $DEEPSEEK_API_KEY"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
echo "PATH: $PATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "============================================="