#!/bin/bash
echo "=============================="
echo " ROS2 Development Environment "
echo "=============================="

echo -n "OS Info: "
uname -a 

# Check and print Python version
if command -v python3 &> /dev/null; then
    echo -n "Python Version: "
    python3 --version 
else
    echo "Python Version: Not Found"
fi

# Check and print gcc version
if command -v gcc &> /dev/null; then
    echo -n "Gcc Version: "
    gcc --version | head -n 1 
else
    echo "Gcc Version: Not Found"
fi

# Check and print CMake version
if command -v cmake &> /dev/null; then
    echo -n "CMake Version:"
    cmake --version | head -n 1 
else
    echo "CMake Version: Not Found"
fi

echo "ROS Distro: $ROS_DISTRO"

echo "============================="

# 可选：显示当前 Python 虚拟环境 (如果激活了)
if [ -n "$VIRTUAL_ENV" ]; then
  echo "Active VENV: $VIRTUAL_ENV"
  echo "============================="
fi