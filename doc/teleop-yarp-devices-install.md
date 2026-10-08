# Installation from Source Code

First install the dependencies:
- [Install CMake 3.19+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-ycm.md/)
- [Install YARP 3.12+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-yarp.md/)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-googletest.md/).

## Installation (Ubuntu)

Once the required dependencies have been installed, the code has to be compiled and installed. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/teleop-yarp-devices.git  # Download teleop-yarp-devices software from the repository
cd teleop-yarp-devices; mkdir build; cd build; cmake ..  # Configure the teleop-yarp-devices software
make -j$(nproc)  # Compile
sudo make install  # Install :-)
sudo ldconfig  # Just in case
```

Remember to enable the devices you want to compile using `ccmake` instead of `cmake`.
