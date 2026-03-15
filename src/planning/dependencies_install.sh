sudo apt-get install libcgal-dev -y
sudo apt-get install libgsl-dev -y

# OSQP with Eigen wrapper for C++ planning
sudo apt-get install -y libeigen3-dev git cmake build-essential

# Install OSQP
cd /tmp
git clone --recursive https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install

# Install osqp-eigen
cd /tmp
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make
sudo make install