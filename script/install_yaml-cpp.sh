#git clone https://github.com/jbeder/yaml-cpp.git
git clone https://gitee.com/mirrors/yaml-cpp.git
cd yaml-cpp
mkdir build

cd build
cmake -DBUILD_SHARED_LIBS=ON ..

make
sudo make install
sudo ldconfig
