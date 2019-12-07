#!/bin/sh

ubuntu_version=$(lsb_release -r)
ubuntu_version=$(cut -f2 <<< "$ubuntu_version")

mkdir ~/.mico/ -p
cd ~/.mico/
mkdir thirdparty -p
cd thirdparty

install_git_repo () {
	if [ -d "./$1" ] 
	then
		echo "Library $1 already installed" 
	else
		git clone $2
		cd $1
		if [ -z "$3" ]   # Is parameter #1 zero length?
		then
			git checkout "$3"
		fi
		
		mkdir build ; cd build
		cmake ..
		make -j$(nproc)
		sudo make install 
		cd ../..
	fi
}


###################################################################
###########		INSTALL CMAKE LAST RELEASE		        ###########
###################################################################

sudo apt-get install -y libssl-dev
if [ -d "CMake" ] 
then
	echo "Library $1 already installed" 
else
	git clone -b "release" "https://github.com/Kitware/CMake"
	cd CMake
	./bootstrap ; make ; sudo make install
	cd ..
fi

sudo apt-get install -y libboost-all-dev curl

sudo apt-get install -y qt5-default
sudo apt-get install -y libqt5opengl5 libqt5opengl5-dev

install_git_repo "nodeeditor" "https://github.com/paceholder/nodeeditor"

###################################################################
###########				INSTALL CUDA 10 		        ###########
###################################################################

if [ "$ubuntu_version" == "16.04" ]; then
	# Add NVIDIA package repositories
	# Add HTTPS support for apt-key
	sudo apt-get install -y gnupg-curl
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_10.0.130-1_amd64.deb
	sudo dpkg -i cuda-repo-ubuntu1604_10.0.130-1_amd64.deb
	sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
	sudo apt-get update
	wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
	sudo apt install ./nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
	sudo apt-get update
	# Install NVIDIA driver
	# Issue with driver install requires creating /usr/lib/nvidia
	sudo mkdir /usr/lib/nvidia
	sudo apt-get install -y --no-install-recommends nvidia-418
	# Reboot. Check that GPUs are visible using the command: nvidia-smi
	# Install development and runtime libraries (~4GB)
	sudo apt-get install -y --no-install-recommends \
		cuda-10-0 \
		libcudnn7=7.6.2.24-1+cuda10.0  \
		libcudnn7-dev=7.6.2.24-1+cuda10.0
	# Install TensorRT. Requires that libcudnn7 is installed above.
	sudo apt-get install -y --no-install-recommends libnvinfer5=5.1.5-1+cuda10.0 \
		libnvinfer-dev=5.1.5-1+cuda10.0
fi;
if [ "$ubuntu_version" == "18.04" ]; then
	# Add NVIDIA package repositories
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
	sudo dpkg -i cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
	sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
	sudo apt-get update
	wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
	sudo apt install ./nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
	sudo apt-get update
	# Install NVIDIA driver
	sudo apt-get install -y --no-install-recommends nvidia-driver-418
	# Reboot. Check that GPUs are visible using the command: nvidia-smi
	# Install development and runtime libraries (~4GB)
	sudo apt-get install -y --no-install-recommends \
		cuda-10-0 \
		libcudnn7=7.6.2.24-1+cuda10.0  \
		libcudnn7-dev=7.6.2.24-1+cuda10.0
	# Install TensorRT. Requires that libcudnn7 is installed above.
	sudo apt-get install -y --no-install-recommends libnvinfer5=5.1.5-1+cuda10.0 \
		libnvinfer-dev=5.1.5-1+cuda10.0
fi;	

###################################################################
###########		INSTALL OPENCV and OPENCV contrib		###########
###################################################################


if [ -d "opencv" ] 
then
	echo "Library $1 already installed" 
else
	git clone "https://github.com/opencv/opencv_contrib"
	cd opencv_contrib 
	git checkout "3.3.1"
	git clone "https://github.com/opencv/opencv"
	cd opencv
	git checkout "3.3.1"
	mkdir build; cd build
	cmake .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DENABLE_PRECOMPILED_HEADERS=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DWITH_NVCUVID=OFF -DBUILD_opencv_cudacodec=OFF
	make -j$(nproc)
	sudo make install 
	cd ../..
fi


###################################################################
###########					INSTALL PCL	 				###########
###################################################################

if [ "$ubuntu_version" == "16.04" ]; then 
	sudo apt-get install -y libeigen3-dev libflann-dev libvtk6-dev libqhull-dev
	install_git_repo "pcl" "https://github.com/PointCloudLibrary/pcl"
fi;

if [ "$ubuntu_version" == "18.04" ]; then 
	sudo apt-get install -y libpcl-dev
fi;


###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################
sudo apt-get install -y liblapack-dev libblas-dev libopenblas-dev libeigen3-dev
sudo apt-get install -y libomp-dev libsuitesparse-dev libcholmod3

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"

install_git_repo "DLoopDetector" "https://github.com/dorian3d/DLoopDetector"

###### INSTALL G2O ################################################
if [ -d "g2o" ] 
then
	echo "Library $1 already installed" 
else
	git clone "https://github.com/RainerKuemmerle/g2o"
	cd g2o
	mkdir build; cd build
	cmake -DBUILD_WITH_MARCH_NATIVE=ON .. 
	make -j$(nproc)
	sudo make install 
	cd ../..
fi


###################################################################
###########				INSTALL MAVSDK					###########
###################################################################
# install_git_repo "MAVSDK" "https://github.com/mavlink/MAVSDK"

install_git_repo "flow" "https://github.com/Bardo91/flow"
