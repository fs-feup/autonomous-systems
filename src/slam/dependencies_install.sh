# for Graph SLAM, needed for GTSAM
sudo apt-get install libboost-all-dev
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && source ~/.bashrc # Add this library path to the system
cd ./ext/gtsam && mkdir build && cd build && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_PREFIX_PATH="/usr/share/eigen3" .. && sudo make check && sudo make install && cd ../../..