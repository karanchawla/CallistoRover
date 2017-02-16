# CallistoRover
Repository for the software for control and autonomous navigation of UGV.
Directory Structure: 
- CallistoRover
  - src 
    - callisto_control
      - config 
        callisto_control.yaml
      - launch 
        - callisto_control.launch
      - CMakeLists.txt
      - package.xml
    - callisto_description 
      - launch 
        - callisto_rviz.launch
      - urdf 
        - macros.xacro 
        - materials.xacro 
        - callisto.gazebo
        - callisto. xacro 
       - CMakeLists.txt
       - package.xml
     - callisto_gazebo
       - launch
         - callisto_myworld.launch
       - worlds
         - callisto.world
       - CMakeLists.txt
       - package.xml
     - diffdrive_controller
       - src
         - diffdrive_controller.py
       - CMakeLists.txt
       - package.xml
     - nav_behaviours
       - launch 
         - nav_behaviours.launch
       - src
         - nav_timed.py
       - CMakeLists.txt
       - package.xml
       
Working of the LIDAR:
![](/images/laser.png)

To Do:
- [x] Get the connections for Sabertooth working
- [x] Simulate the robot in Gazebo
- [x] Add differential drive controller and, camera sensor.
- [x] Interface mbed with ROS using Serial Comms 
- [x] Open Loop Control Node for mbed 
- [ ] Implement HIL open loop control node
- [x] Use encoder data for PID controller - feedback!
- [ ] Figure out VICON stuff 
- [ ] Interface VICON with rover 
- [ ] Trajectory Tracking 
Okay this is getting intimidating - rest of it later!
 
Notes: 
 - 1. If Hokuyo laser shows weird range and behavior, the problem is probably because of the graphic drivers and add this line before launching gazebo simulation - you may add it to your ~/.bashrc file. 
 
 ```
 export LIBGL_ALWAYS_SOFTWARE=1
 ```
 
- 2. Converting from ogv to mp4

'''
 ffmpeg -i out.ogv        -c:v libx264 -preset veryslow -crf 22        -c:a libmp3lame -qscale:a 2 -ac 2 -ar 44100 \ navigation.mp4

'''
