# for Graph SLAM, needed for GTSAM
sudo apt-get install libboost-all-dev

cd ./ext/gtsam && mkdir build && cd build && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_PREFIX_PATH="/usr/share/eigen3" .. && sudo make check && sudo make install && cd ../../..

pip install transforms3d==0.4.1
pip install scipy==1.10.0