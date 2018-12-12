echo "Build ROS node ..."

cd Examples/ROS/ORB_VIO
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
cd ../../../../
