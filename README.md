# Master Thesis (Jacobs University Bremen) [WIP]
## A Large Scale Realtime 3D Reconstruction Pipeline for 3D mapping

### Description
The aim of this thesis is to build a large scale 3D reconstruction pipeline, with applications for realtime 3D mapping. Note: **Currently a work in progress**.

The project contains 4 essential components:

1. A *stereo streamer*: this will be developed as a ROS component that streams stereo images, as well as IMU data.

2. *3D Reconstruction Server*: A server that will accept the stereo stream from the *stereo streamer* and reconstruct point clouds in **real-time** using SLAM techniques.

3. *Unreal Engine Visualisation*: An Unreal Engine client that connects to the reconstruction server and visualises the 3D map in **real-time**.

4. *Networking Protocol*: The networking protocol built using C++14, and Boost.Asio as an independant library to connect all 3 components outlined above together.

### Technoliges Used
The full pipline will utlise modern C++ as the language choice, with CMake used as a build system.
