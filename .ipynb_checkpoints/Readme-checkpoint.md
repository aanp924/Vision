## VISION Dog project: 
    A robotic dog on wheels that could guide a visually impaired person along a generated path, avoiding obstacles 
I built off of an idea from elementary school, coming out with an improved project after I got the knowledge and tools I worked on object detection and classification, a significant part of generating a safe path
I decided to use the efficient YOLO algorithm because of its single pass as a neural network 
I manually labelled/annotated objects to train the YOLO model and used the A* algorithm to find the shortest path 
I added waypoints for a smoother curve and experimented with the Stanley method, which is commonly used in autonomous vehicles
This approach was adding latency and timely processing of video frame was affected. So, I decided to use IMU data and applying  filter( Kalman, Extended Kalman) for precise state estimation and control. This data is fed to PID loop keeping the robot moving with constant velocity
There is a camera attached for video data input in addition to ultrasonic sensors and IMU
I used an esp32 processor as a baseline but it wasn’t able to efficiently process all of the data, so I transitioned to a raspberry pi 5 processor and eventually an expansion board as well I am also using metal-geared encoded motors for more torque, helping move the robot despite its weight

### Future Goals/Ideas:
The goal of this project is to prevent tragedies like one that happened in my own neighborhood where a visually impaired man was run over by a car
I want to use mapping of  real world objects coordinates to image coordinates and use depth sense capable camera to enhance capabilities
I want to use data from camera and IMU to build fusion model for localizations and environmental awareness which will help control robot more precisely and accurately
I want to have a “leash and a handle” attached to provide audio and vibration feedback to indicate obstacles, elevation changes, etc where users would hold the handle for comprehensive feedback about surrounding environment
I want to put together a 4 - legged robot dog and a more sturdy, space efficient dog frame, maybe metal or 3D printed, but the hardware was difficult to gain access to for the prototype, so I started with wheels on the robot 
I want to extend the platform with multi modal sensors - camera, ultrasonic sensors, LiDAR, IMU, and use their data to build fusion models which  opens up various research advancement and applications possibilities, in my project focused on human centric development building technology aids.

