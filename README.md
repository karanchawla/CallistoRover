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

Videos:

<a href="https://www.youtube.com/watch?v=vewtGyf9uSo&t=2s" target="_blank"><img src="https://i.ytimg.com/vi/vewtGyf9uSo/hqdefault.jpg?custom=true&w=336&h=188&stc=true&jpg444=true&jpgq=90&sp=68&sat=0.3&sigh=OgGL06-jMYgpFNx8oOrUetsa3hc" alt="Teleoperation and Mapping" width="240" height="180" border="10" /></a>


<a href="https://www.youtube.com/watch?v=-S2_TLgltj8" target="_blank"><img src="https://i.ytimg.com/vi/-S2_TLgltj8/hqdefault.jpg?custom=true&w=336&h=188&stc=true&jpg444=true&jpgq=90&sp=68&sat=0.3&sigh=jG1sKAO51Nnar-ijQ_CPH6xkco0" alt="Autonomous Navigation with AMCL" width="240" height="180" border="10" /></a>


<a href="https://goo.gl/photos/nmXYycaKVH1yMC3G9" target="_blank"><img src="https://lh3.googleusercontent.com/LTIWOhX_Uc8lWjo0i4cZI0cqgiaAOq_9a7IJA8IKMLfwu8WBUfPZ3q7BbmeAlETHdsNsjyJSfw8_5aupvpemRNS2hOoiB5ejP1j8xIrQZiFH1xLjqF39teBuP9KLyOQk2xzSDP5eRxrxFP14n7w_aHa9X11ty8CXGvkylqNhpho9ltcAuOUAPDnTCalrStW9yyi08DpG3F5MXiaJ0dTwMJMVDUCzb0qJC9V0V-3E_DAblD3syx-rnxvPf-BgAW8PyGNR71rkNmJSBpCPjOr23RmcYgueybx5Qa_af31Frx8M683foQDHeNOWw1GTJlynsw9zuVFf-LXdYvNAkeEOZQkuVROOa5RXga98fI81Ac9b1nCVCnhTtFbNbw8ijjLTUowz3O5tazoVLShzjk5FwbT45Kh0__SXjpZEA6F4y1-aJDYpTd_fyxKHdDSAheEGMstbNs8v5mxL6CyAJSlWvWil34kxfF2O1-JffLwBvDobRHQXD9G_jreS1V4pFn3sbElk5tCblkrN3r7GjmHqo8sZb1IYfvMgYdyWECR3YM1TSiUA2JIcuVf5AwA_roveQthg3Zt_jDVlsK6CJ0ccDqzk9_eom3r6CIolgYVKAdnQwZgU6YD2Pw=w1713-h963-no" alt="Trajectory Tracking" width="240" height="180" border="10" /></a>

 
![]()
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

```
ffmpeg -i out.ogv        -c:v libx264 -preset veryslow -crf 22        -c:a libmp3lame -qscale:a 2 -ac 2 -ar 44100 \ navigation.mp4

```
