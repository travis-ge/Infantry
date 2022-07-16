sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev
### 可能会出现无法定位libcxsoarse3.1.2的问题
#//第一步，打开sources.list
# sudo gedit /etc/apt/sources.list
#//第二步，将下面的源粘贴到最上方sources.list
# deb http://cz.archive.ubuntu.com/ubuntu trusty main universe
#//第三步，更新源
# sudo apt-get update
# 第四步，重新输入依赖项安装命令安装依赖项

git clone https://gitee.com/mirrors/ceres-solver.git

cd ceres-solver
mkdir build
cd build

sudo cmake ..
sudo make -j4
sudo make test
sudo make install
