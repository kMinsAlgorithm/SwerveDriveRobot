# Swerve-Drive-Robot

- A swerve drive is an advanced wheel configuration in which each wheel module can rotate independently and drive under its enabling omnidirectional movement, zero-radius turns, and exceptional maneuverability.


<div style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/566b886d-32d5-46ef-b57a-b08568042302" 
       alt="swerve_motion"
       style="width: 60%; height: auto;">
</div>


- In this project, we designed three custom swerve modules, each equipped with a motor for steering and another for propulsion.
- The orientation of each drive module is measured using an angle sensor, and its steering motor is controlled by a PID controller to accurately achieve the target angle.


<div style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/c9ba42b5-cc77-4cf1-a0be-b62b1f55fde9" 
       alt="swerve_design"
       style="width: 60%; height: auto;">
</div>



- An Arduino Mega simultaneously computes the steering angles and wheel speeds to perform real-time odometry estimation.
- It then publishes the odometry messages over the CAN bus in real time.
- The entire system is operated under the ROS 2.

<div style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/60ec75e0-e73d-4e9e-88d1-152c88caf6da" 
       alt="control_structure"
       style="width: 60%; height: auto;">
</div>


For a detailed explanation, please refer to the following link:  
[Swerve Drive Documentation](https://kminsalgorithm.github.io/robot1.html)
