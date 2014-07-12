#!/bin/sh
#Installation script for aruco 
rm -rf aruco # Clearing up existing installations
mkdir aruco
cd aruco
wget -O aruco.tgz http://sourceforge.net/projects/aruco/files/latest/download?source=files
tar -xf aruco.tgz --strip-components 1 #Untar files and remove the parent directory
mkdir build && cd build && cmake .. # Add sophisticated error finding mechanisms
make -j 3 && sudo make install
