#! /bin/bash

# rslidar_sdk
if test -f ./ext/rslidar_sdk/dependencies_install.sh; then
    sudo chmod u+x ./ext/rslidar_sdk/dependencies_install.sh
    ./ext/rslidar_sdk/dependencies_install.sh
fi

# kvaser - does not work in docker
sudo apt-get -y install wget build-essential pkg-config gcc-12
# Download and compile the latest Kvaser linuxcan driver
wget -O linuxcan.tar.gz "https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
tar xvzf linuxcan.tar.gz
cd linuxcan
make
sudo make install 
sudo make load
cd ..
# Optional SDK libraries (kvlibsdk) - uncomment if needed for Kvaser's proprietary API later
# sudo apt-get install -y libxml2-dev zlib1g-dev
# wget -O kvlibsdk.tar.gz "https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130981966&utm_status=latest"
# tar xvzf kvlibsdk.tar.gz
# cd kvlibsdk
# make
# make check
# sudo make install
# cd ..
# Cleanup
rm -rf linuxcan
rm -rf kvlibsdk
rm -f linuxcan.tar.gz
rm -f kvlibsdk.tar.gz