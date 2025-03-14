# for Graph SLAM, needed for GTSAM
sudo apt-get install libboost-all-dev libmetis-dev
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && source ~/.bashrc # Add this library path to the system
cd ./ext/gtsam && mkdir build && cd build && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_PREFIX_PATH="/usr/share/eigen3" -DCMAKE_BUILD_TYPE=Release -DGTSAM_WITH_TBB=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ENABLE_NESTED_DISSECTION=ON -DGTSAM_WITH_CUDA=OFF -DCMAKE_CXX_FLAGS="-march=native -O3 -DNDEBUG -funroll-loops" .. && sudo make check && sudo make install && cd ../../..