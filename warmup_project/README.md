# Warmup Project
## Luke Nonas-Hunter | Sam Kaplan
### Computational Robotics Fall 2020


This is the first project of Introduction to Computational Robotics at Olin College of Engineering. 

In order to get familiar with ROS, the rospy library, object oriented code structure, LIDAR perception, reactive control, and proportional control we completed six robot behavior routines on a simulated Neato robot: 


* [Teleoperation](#teleoperation)
* [Driving a Square](#driving-in-a-Square)
* [Wall Follower](#following-a-Wall)
* [Person Follower](#Follow-a-Person)
* [Obstacle Avoidance](#Avoid-Obstacles)
* [Finite-State Controler](#finite-state-controller-implementation)

> Note: this class was supposed to provide more concrete robot feedback with real, physical robots, but due the COVID-19 pandemic it was converted to use the Gazebo simulator, which, for the most part, was an accurate representation of the real thing. 


## Teleoperation

    For this task, we wrote a simple teleoperation node that allows the user to control the robot with keyboard input. As it stands now, the command structure is as follows: 

    
| key | lin_vel   | ang_vel   | action  |
|:---:|:---------:|:---------:|:-------:|
|  w  |    0.3    |    0      | forward |
|  a  |    0      |    0.3    | turn left|
|  s  |    -0.3   |    0      | turn right |
|  d  |    0      |    0.3    | backward  |
|  e  |   0       |    0      | stop  |
  
    As you can see, this is a very bare bones implementation, much simpler than the provided [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) . We mostly wanted to get this done with so we could focus on the more complicated behaviors later in the project. 

    Key Takeaway: Non-clunky robot control is hard. If we had more time, we would have implemented a combined linear-angular velocity feature. This would have allowed us to arc for a turn, instead of stop motion all together in order to turn to the desire angle. Also good to know that an ESTOP function that take a keyboard imput can be implemented if desired (probably more import when testing on real robots), instead of simply cancelling the running node in terminal. This can cause all sorts of issues, most notably that the robot will keep acting on its last received cmd_vel. 

## Driving in a Square

    The two seemingly best ways to instruct our robot to drive in 1m x 1m square were as follows: command a specified velocity for a given time, then turn 90 degrees, or to check encoder data and turn using odometry. We chose to write this behavior using odometry because learning to read odometry data is infinitely more useful in the long run than what is essentiall hardcoding straight-aways and turns using time.

    The majority of time spent on this behavior trying to figure out how to interpret the data we were given by our odometry subscriber. Our lives were made significanly easier by the discovery of the transformations.py library, which allowed us to convert from the given quaternions to simple Euler angles, given a specifed axis order.

    After that, it was simply a matter or converting the Euler angles into a convient span of angles. Unfortunely, we kept the default span of, which ranged from -PI to PI, instead of shifting over to a 0-2PI span. This caused headaches hardcoding later on which could have easily been avoided.

    Key Takeaway: 

## Following a Wall

    Wall following is a simple, but important reactive behavior to implement. Much of the later behaviors would be influenced by how we decided to tackle this problem. 


    Out implementation takes the shortest distance of a given set of LaserScan data as the angle it wants to orient itself 90 degrees from. The robot will always want to turn the shortest distance, so the direction it travels parallel to the wall in dependent on its initial orientation. If it's right side is wall-facing, it will travel that way, and vice versa. 

    maybe insert a diagram or something?

    Key Takeaway: Learn to switch to a different approach when a particular technique doesn't seem to be working. We believe our logic was sound, but we must have been missing something in the Python implementation. However once we switch to a method that was easier to visualize regarding the laserscan data, the behavior worked flawlessly. In addition, it turned out that by restructuring our code into a getter | setting protocol, we made our code more robust and reusable.

## Follow a Person

    Following an person is another fundamental behavior for any reactive contronl robot. And indeed, out of all the routines made for the project, it is the most human-like. For isn't it human nature to seek companionship? To find another being with which we can share our lives?

    The robot does not want to get that close, however. In fact, the distance that it will follow you around can be specified. 

    Key Takeaway: Don't look for error, have error influence movement in such a way that zero error will be zero movement. Want to implement a param to quickly adjust minutia of robot behavior. 

## Avoid Obstacles

    Avoiding obstacles is one of the most important behaviors for any robot wishing to operate inside an environment with objects. Assuming most robots don't operate in the middle of a flat, featureless plane, it is safe to say that obstacle detection is a priority of robot operation.

    Key Takeaway: rotation matrices are difficult to implement. Our largest code structure of the project by far, is much harder to keep track of. Stress the importance of documentation!

## Finite-State Controller Implementation

    Fairly straightforware using the smach library. Wanted to use smach because it seems easily scalable, industry standard, and can run multiple machines in paralell, which is a much for robots for complicated than a single differential drive system. 

    The only thing needed to be added was a helper function to tell when to switch states. Because one state operated without the need to track an object, and object tracking was inherent in the other behavior's function, it seemed like a good idea to have the state machine switch be whether or not the laser scan was receiving "real" data, i.e. real numbers. 