# Swerve-Drive-Robot

-A swerve drive is an advanced wheel configuration in which each wheel module can rotate independently and drive under its enabling omnidirectional movement, zero-radius turns, and exceptional maneuverability.

- In this project, we designed three custom swerve modules, each equipped with a motor for steering and another for propulsion.

- The orientation of each drive module is measured using an angle sensor, and its steering motor is controlled by a PID controller to accurately achieve the target angle.

- An Arduino Mega simultaneously computes the steering angles and wheel speeds to perform real-time odometry estimation.
- It then publishes the odometry messages over the CAN bus in real time.
- The entire system is operated under the ROS 2.

This work was carried out as part of my M.S.
                            research and was supported by a National
                            Research
                            Foundation of Korea (NRF) <strong>"Semantic
                                Autonomous Driving Systems Utilizing Diffusion
                                Models for Enhanced Dynamic Obstacle Avoidance
                                Performance"</strong>.
