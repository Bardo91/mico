![](https://github.com/Bardo91/mico/blob/master/doc/mico_banner.png)

**M**odular **I**nterchangeable **C**omputer Visi**O**n

[![Releases](https://img.shields.io/github/release/bardo91/mico.svg)](https://github.com/bardo91/mico/releases)  [![Issues](https://img.shields.io/github/issues/bardo91/mico.svg)](https://github.com/bardo91/mico/issues)  [![Stars](https://img.shields.io/github/stars/bardo91/mico.svg)](https://github.com/bardo91/mico/stars)

[![Build Status](https://travis-ci.org/Bardo91/mico.svg?branch=master)](https://travis-ci.org/Bardo91/mico)

# Installation

Clone the repository
```
https://github.com/Bardo91/mico
```

Get in and install dependencies
```
cd mico
bash script/install_dependencies.sh
```

Compile mico and install
```
mkdir build &&  cd build
cmake ..
make -j$(nproc)
sudo make install
```

# Tested Build configurations

|   | Ubuntu  | OpenCV | OpenCV_contrib | PCL                          | CUDA | g2o |   |
|---|---------|--------|----------------|------------------------------|------|-----|---|
| 1 | 18.04.1 | 3.3.1  | 3.3.1          | 1.8.1+dfsg1-2ubuntu2.18.04.1 | 10.0 | 1.0 |   |



### Example odometry RGBD
[![IMAGE](http://i3.ytimg.com/vi/WctJBLkTNro/hqdefault.jpg)](https://youtu.be/WctJBLkTNro)

### Example odometry RGBD with point cloud visualization
[![IMAGE](http://i3.ytimg.com/vi/G-6rvcv5ks8/hqdefault.jpg)](https://youtu.be/G-6rvcv5ks8)
