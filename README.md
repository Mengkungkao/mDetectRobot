# mDetectRobot

1. sudo apt-get update && sudo apt-get upgrade && sudo rpi-update
2. sudo nano /etc/dphys-swapfile
    CONF_SWAPSIZE=2048
3. sudo apt-get install build-essential cmake pkg-config

   
5. sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
number 5 error
   
7. sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
8. sudo apt-get install libxvidcore-dev libx264-dev
9. sudo apt-get install libgtk2.0-dev libgtk-3-dev
10. sudo apt-get install libatlas-base-dev gfortran
11. wget -O opencv.zip https://github.com/opencv/opencv/archive/4.1.0.zip
12. wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.1.0.zip
13. unzip opencv.zip
14. unzip opencv_contrib.zip
15. sudo pip3 install numpy
16. cd ~/opencv-4.1.0/
17. mkdir build
18. cd build
19. cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-4.1.0/modules \
  -D BUILD_EXAMPLES=ON ..
20. make -j4
21. sudo make install && sudo ldconfig
22. sudo reboot





