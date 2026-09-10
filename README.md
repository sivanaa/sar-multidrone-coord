#sar-multidrone-coord
 Multi-drone coordination layer for the SAR drone project.
>
> Adds a ROS 2 coordination layer (one 'coordination_node' per drone) on top of the existing single-drone SAR pipeline (Jetson Orin NX + Pixhawk?MAVLink, Rust/Axum gateway, YOLOv8/ResNet-18/U-Net vision models), that pipeline is unchanged.
>
> -Drones bid on identifying a person/target and report coordinates to each other over ROS 2 topics/DDS (pose, region status, target-found, assist requests, task reassignment).
> -Designed generically for N drones; prototyped at N=5 in simualtion, real deployment target is 2.
> -Simulation:validate in Gazebo Harmonic + PX4 SITL first, port to AirSim later for vision-pipeline testing against more realistic imagery.

See ros2_ws/ for the ROS 2 workspace (built on the gpu server). 
