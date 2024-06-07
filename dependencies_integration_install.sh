#! /bin/bash
# rslidar_sdk
if test -f ./ext/rslidar_sdk/dependencies_install.sh; then
    sudo chmod u+x ./ext/rslidar_sdk/dependencies_install.sh
    ./ext/rslidar_sdk/dependencies_install.sh
fi

# kvaser - does not work in docker
sudo apt-get -y install wget
sudo apt-get -y install build-essential
sudo apt-get -y install pkg-config 
sudo apt install gcc-12
wget --content-disposition "https://resources.kvaser.com/PreProductionAssets/Product_Resources/linuxcan_5_45_724.tar.gz"
tar xvzf linuxcan_5_45_724.tar.gz
cd linuxcan
make
sudo make install 
# sudo make load
# cd ..
# sudo apt install -y libxml2-dev
# sudo apt install -y zlib1g-dev
# wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
# tar xvzf kvlibsdk.tar.gz
# cd kvlibsdk
# make
# make check
# sudo make install
# cd ..
# rm -rf linuxcan
# rm -rf kvlibsdk
# rm linuxcan.tar.gz
# rm kvlibsdk.tar.gz
# rm linuxcan.tar.gz.1
# rm kvlibsdk.tar.gz.1
rm linuxcan_5_45_724.tar.gz
rm linuxcan_5_45_724.tar.gz.1