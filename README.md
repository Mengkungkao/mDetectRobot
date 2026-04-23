# mDetectRobot

1. sudo apt-get update && sudo apt-get upgrade && sudo rpi-update
2. sudo nano /etc/dphys-swapfile
    CONF_SWAPSIZE=2048
3. sudo apt-get install build-essential cmake pkg-config
4. sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
5. sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
6. sudo apt-get install libxvidcore-dev libx264-dev
7. sudo apt-get install libgtk2.0-dev libgtk-3-dev
8. sudo apt-get install libatlas-base-dev gfortran
9. wget -O opencv.zip https://github.com/opencv/opencv/archive/4.1.0.zip
10. wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.1.0.zip
11. unzip opencv.zip
12. unzip opencv_contrib.zip
13. sudo pip3 install numpy
14. cd ~/opencv-4.1.0/
15. mkdir build
16. cd build
17. cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-4.1.0/modules \
  -D BUILD_EXAMPLES=ON ..
18. make -j4
19. sudo make install && sudo ldconfig
20. sudo reboot





