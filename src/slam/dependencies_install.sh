# for Graph SLAM, needed for GTSAM
sudo apt-get install libboost-all-dev libmetis-dev # TODO: put all dependencies in files like this and make the main script call these files
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && source ~/.bashrc # Add this library path to the system
cd ./ext/gtsam && mkdir build && cd build && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_PREFIX_PATH="/usr/share/eigen3" -DCMAKE_BUILD_TYPE=Release -DGTSAM_WITH_CUDA=OFF .. && sudo make check -j4 && sudo make install -j4 && cd ../../..
colcon build --packages-select gtsam 