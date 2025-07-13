#!/bin/bash
set -e

echo "Updating package lists..."
sudo apt-get update

echo "Installing protobuf compiler and C++ libraries..."
sudo apt-get install -y protobuf-compiler libprotobuf-dev

echo "Installing Python protobuf and GUI dependencies..."
pip3 install --upgrade pip
pip3 install protobuf pyqt5 pyqtgraph pyserial

echo "All dependencies installed!" 