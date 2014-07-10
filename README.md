Robo Blender
============

This Blender rig maps onto ROS controllers to steer the Hanson Robotics Zeno robot body

It now uses the motors.yaml file to configure the connection between the joints and the ROS publish points.
It uses anim.yaml to read key frame animation frame ranges
It uses inputs.yaml for face motion .. currently I plan to remove or seperately use face animation from blender( it needs some new logic .. maybe need to keep categories of animations(face, legs, torso) and play them on cammand)

The main concept here is to read animation commands from ros as a string(mapped with anim.yaml), set blender to compute the keyframe and all joint positions, then collect the joint trajectories(position,velocity) and send thm to respective motor controllers. It will be able to slow down animations upto 1Hz(custom duration) as specified, and has a minimum time that animation needs to complete specified by animator. animations could be played slower but would not always be desirable like in case of walking.

Currently the system supports bones but support is being added for shape keys.
