# SPQR Team Code used in RoboCup 2024 based on B-Human 2021 Code Release

# Changelog

## 🆕 Added
   ➕ Referee's gesture recognition model \
   ➕ Whistle detection model 

## 🔄 Changed
   🔧 Ball perceptor \
   🔧 Behaviors

## 🐛 Fixed
## ❌ Removed

# Installation

### Supported distros:
* Ubuntu 20.04, 22.04
* Linux Mint 18.x / 20.x

## Install required dependencies
Open a terminal `Ctrl`+`Alt`+`t` (usually) and type the followings commands: <br>
* `$ sudo apt install clang cmake git graphviz libasound-dev libglew-dev libqt5opengl5-dev libqt5svg5-dev lld llvm net-tools ninja-build pigz qtbase5-dev rsync wish xterm xxd` 

### Install dav1d
* **Ubuntu 21.10 or higher** (only with APT): `$ sudo apt install libdav1d-dev`

* **otherwise**, you'll have to compile and install from source. The repo is found at https://code.videolan.org/videolan/dav1d

  1. Clone the repo: `git clone https://code.videolan.org/videolan/dav1d.git`
  2. Enter the new folder: `$ cd dav1d`
  3. Make a build directory: `$ mkdir build && cd build`
  4. Configure meson: `$ meson ..`
  5. Compile: `$ ninja`
  6. Install: `$ sudo ninja install`
  7. Trick your system into believing you have the correct version (1 of 2): `$ cd /usr/local/lib/x86_64-linux-gnu`
  8. (2 of 2): `$ sudo ln --symbolic libdav1d.so.6.7.0 libdav1d.so.4`
  9. Close this terminal or use `cd` to change directory again, whatever you do don't remain in this library path by accident.



### Configure GitHub :
  * Install git - `$ sudo apt-get install git git-core git-gui git-doc`
  * Install gitg - `$ sudo apt-get install gitg`
  * Set up git you can do that by reading this [page](http://help.github.com/linux-set-up-git) (Attention: "Your Name here" means "Nome Cognome")
  * Colorize git: `$ git config --global color.status auto`


### Important: Update and Upgrade your OS
* `$ sudo apt-get update`
* `$ sudo apt-get upgrade`
* `$ sudo reboot`

## Clone the repo
  * Create a folder called RoboCup in your home folder - `$ mkdir RoboCup`
  * Enter in the RoboCup folder - `$ cd RoboCup`
  * Execute the command to clone - `$ git clone https://github.com/SPQRTeam/spqr2024.git`


### Install OpenCV 3.4

* Download source of OpenCV
```
wget https://github.com/opencv/opencv/archive/3.4.1.zip
```
* Unzip the files
```
unzip 3.4.1.zip
```
* Move into OpenCV directory
```
cd opencv-3.4.1
```
* Create the build directory 
```
mkdir build && cd build
```
* Generate the make file
```
cmake -DWITH_QT=OFF -DWITH_OPENGL=OFF -DBUILD_SHARED_LIBS=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DWITH_CUDA=OFF -DQT_NO_VERSION_TAGGING=ON -DWITH_PNG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENJPEG=OFF -DWITH_JASPER=OFF -DWITH_OPENEXR=OFF -DBUILD_TBB=ON ..

```
* Open `opencv-3.4.1/modules/python/src2/cv2.cpp`. Move to line 889 and replace `char* str = PyString_AsString(obj);` with `const char* str = PyString_AsString(obj);` 

* Compile
```
make -j#num of core of your cpu
```
* Install OpenCV
```
sudo make install
```
and then
```
sudo ldconfig
```

## Building the code


### Generate
**Whenever a file is added/removed/modified**, we need to regenerate the CMakeLists file with:
  ```
  NO_CLION=True path/to/repo/root/Make/Linux/generate 
  ```

### Compile
  ```
  path/to/repo/root/Make/Linux/compile
  ```

### Bash script

It might be beneficial to create a bash script that does this automatically:
  ```
  #! /bin/bash

  cd spqr2024    # Go to repo root. REPLACE WITH YOUR OWN PATH.

  touch Make/CMake/Nao.cmake    # notice new symbols in new files
  NO_CLION=true Make/Linux/generate     # notice new files
  Make/Linux/compile     # compile! 
  ```

You could even want to associate this command with a bash alias to speed things up.

</br>
## Possible issues (Ubuntu)

Here's a list of known issues and errors and their respective solutions:

### Generating

* Error: *CMake Error: CMake was unable to find a build program corresponding to "Ninja".  CMAKE_MAKE_PROGRAM is not set.  You probably need to select a different build tool.*
  
  **Solution**:
  1) Install ninja-build
  ```
  sudo apt-get install ninja-build
  ```


### Compiling
* Error: *clang: error: invalid linker name in argument '-fuse-ld=lld'*
  
  **Solution**:
  1) Install llvm
  ```
  sudo apt-get install llvm
  ```
  2) Export its path to 
* GCC version error: if you get an error that looks like a compiler version error (such as "no member named 'strlen' in namespace 'std'")

  The framework requires a GCC version <= 11. You can check what GCC version clang is using by typing 
  ```
  clang -v
  ```  
  Pay attention to the line 
  ```
  Selected GCC installation: /usr/bin/../lib/gcc/x86_64-linux-gnu/XX
  ```
  Where instead of XX there will be the selected GCC version.

