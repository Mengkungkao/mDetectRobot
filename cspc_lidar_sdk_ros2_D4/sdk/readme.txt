
Linux:

1. cd sdk

2. mkdir build

3. cd build

4. cmake ..

5. make

6. ./cspc_lidar /dev/sc_mini -version 4

7. Note:
Linux version has no standard data communication format. In the while loop of the main function, it will keep receiving data. The data is stored in the LaserScan structure. You can use this data according to your project requirements.

