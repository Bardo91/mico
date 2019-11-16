#!/bin/sh

ubuntu_version=$(lsb_release -r)
ubuntu_version=$(cut -f2 <<< "$ubuntu_version")

mkdir ~/.mico/ -p
cd ~/.mico/
mkdir thirdparty -p
cd thirdparty

install_git_repo () {
	read -r -p "Do you want to install $1 [y/N] " response
	if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
	then
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
	fi
}


###################################################################
###########		INSTALL CMAKE LAST RELEASE		        ###########
###################################################################

read -r -p "Do you want to install latest version of CMake [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
	git clone -b "release" "https://github.com/Kitware/CMake"
	cd CMake
	./bootstrap ; make ; sudo make install
	cd ..
fi

sudo apt-get install libboost-all-dev curl

install_git_repo "nodeeditor" "https://github.com/paceholder/nodeeditor"
install_git_repo "flow" "https://github.com/Bardo91/flow"

###################################################################
###########		INSTALL OPENCV and OPENCV contrib		###########
###################################################################

read -r -p "Do you want to install latest version of OpenCV [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
    if [ -d "opencv" ] 
	then
		echo "Library $1 already installed" 
	else
        git clone "https://github.com/opencv/opencv_contrib"
        git clone "https://github.com/opencv/opencv"
        cd opencv
        mkdir build; cd build
        cmake .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
		make -j$(nproc)
		sudo make install 
        cd ../..
    fi

fi

###################################################################
###########					INSTALL PCL	 				###########
###################################################################

if [ "$ubuntu_version" == "16.04" ]; then 
	read -r -p "Do you want to install latest version of PCL [y/N] " response
	if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
	then
		sudo apt-get install -y libeigen3-dev libflann-dev libvtk6-dev libqhull-dev
		install_git_repo "pcl" "https://github.com/PointCloudLibrary/pcl"
	fi
fi;

if [ "$ubuntu_version" == "18.04" ]; then 
	read -r -p "Do you want to install PCL 1.8 from main repositories [y/N] " response
	if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
	then
		sudo apt-get install -y libpcl-dev
	fi
fi;



###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################
sudo apt-get install -y liblapack-dev libblas-dev libopenblas-dev libeigen3-dev
sudo apt-get install -y libomp-dev libsuitesparse-dev libcholmod3

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"

install_git_repo "DLoopDetector" "https://github.com/Bardo91/DLoopDetector"

###### INSTALL G2O ################################################
read -r -p "Do you want to install latest version of G2O [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
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
fi;


###################################################################
###########				INSTALL MAVSDK					###########
###################################################################
install_git_repo "MAVSDK" "https://github.com/mavlink/MAVSDK"

###################################################################
###########				INSTALL KIDS module DEPS		###########
###################################################################

sudo apt-get install -y qt5-default
sudo apt-get install -y libqt5opengl5 libqt5opengl5-dev
#sudo apt-get install -y sudo apt-get install clang-7
#install_git_repo "Catch2" "https://github.com/catchorg/Catch2"