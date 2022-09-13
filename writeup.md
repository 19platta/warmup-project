### Warmup Project - Computational Robotics 2022
Annabelle Platt and Kate Mackowiak

## Teleoperated Neato Driving
For teleop mode we get key inputs and change the angular and linear velocities. The user can input a,s,d,f to control the robot, and hit any other key to stop the robot motion.

## Square Driving
We are using the robots odometry to navigate in a square. We track the robots position in x,y,z coordinates to determine when we have driven forwards 1m. We then check the robots angle compared to its original to determine when it has turned 90 degrees. This is not fully finished, as we have questions for Paul about how quaternions work.

## Wall Follower
For our wall follower we look at the distances measured by the lidar at angles 225 and 315 degrees in the robots coordinate frame. We then turn depending on which of those angles is larger, and continue straight if they are equal indicating that we are perpendicular from the wall. While our robot is a little bit swervy while accomplishing this, it sucessfully follows a wall (albeit in only one direction) and can navigate around corners.