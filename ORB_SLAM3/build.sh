echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DTORCH_PATH=/home/wfram/GCNv2_SLAM/Thirdparty/pytorch/torch/share/cmake/Torch
make -j8