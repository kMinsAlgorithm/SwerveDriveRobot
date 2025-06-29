# Swerve-Drive-Robot

-A swerve drive is an advanced wheel configuration in which each wheel module can rotate independently and drive under its enabling omnidirectional movement, zero-radius turns, and exceptional maneuverability.

- In this project, we designed three custom swerve modules, each equipped with a motor for steering and another for propulsion.

- The orientation of each drive module is measured using an angle sensor, and its steering motor is controlled by a PID controller to accurately achieve the target angle.

<img src="https://github.com/user-attachments/assets/60ec75e0-e73d-4e9e-88d1-152c88caf6da" 
     alt="control_structure"
     style="display: block; margin: 0 auto; width: 60%; height: auto;">



- An Arduino Mega simultaneously computes the steering angles and wheel speeds to perform real-time odometry estimation.
- It then publishes the odometry messages over the CAN bus in real time.
- The entire system is operated under the ROS 2.
![화면 기록 2025-06-29 오전 8 17 14](https://github.com/user-attachments/assets/b8b88825-e830-4a03-a043-545902d32b51)

This work was carried out as part of my M.S.
                            research and was supported by a National
                            Research
                            Foundation of Korea (NRF) <strong>"Semantic
                                Autonomous Driving Systems Utilizing Diffusion
                                Models for Enhanced Dynamic Obstacle Avoidance
                                Performance"</strong>.
