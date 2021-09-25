# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

## Workspace

* Ubuntu 16.04
* PCL - v1.7.2
* C++ v14

1. Install and Run docker desktop
   - [Install Docker](https://www.docker.com/products/docker-desktop)
   - Run docker destkop

2. Pull the image and make the container from kimjw7981/SFND_lidar
   - Check out this page [kimjw7981/SFND_lidar Docker Hub](https://hub.docker.com/repository/docker/kimjw7981/sfnd)
   - Run this command for pull the image and make the container
   ```shell
   docker run -p 6080:80 -v /dev/shm:/dev/shm kimjw7981/sfnd
   ```
   - Connect the linux GUI environment with [localhost:6080](localhost:6080) on your browser

3. Run LX Terminal

4. Execute the following commands in LX Terminal

   ```shell
   sudo apt update -y && sudo apt upgrade -y
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

5. Done
   - Take your time to learn the Sensor Fusion Nanodegree Program

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_macosx.html#compiling-pcl-macosx)
