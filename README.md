
# like_keras

A python library for robotics like keras

A descritve language to describe an complex, autonmous robotics sytem with commerical deployments in mind

Describe it once, configure and run it it many different ways!

- Build around MDP abstraction. If you can describe your problem in terms of sensors, world_model, actor, actuators, you are good!
- Designs we push to run time/ abstract away
- Communication protocol, use Ros, dora-rs, your_fav_new_transport_lib
- Ai first. ackwnoledges that the trend is from many components to more end to end systems, and want to design system in 2-track way that allows you to approximate essentially any part of the system with neural net. Functional Programming/Pipeline programming approach

### Frusterations with existing systems

- using xacro with urdf
- Having to have 5 terminals open, and juggle 10 moving parts
- Ros stack with 50 components


### Moments of inspiration

- The ease of use of pytorch
- The ease of use of gym api
- Robotics moving towards more end to end models, at some point may just be a single connected neural net
- Making launch files with python vs xml, and being able to hack it, add custom functions, etc. (ie use the full power of python)


### Design Goals

- Python first. Forget the yaml config files, lets use python with type hinting and functions. 
- Make Launching the system as easy as running a python script
- "Keep the simple things simple, make the hard things possible"
- Should feel like a well organized workshop. Made for engineers. All the tools in a known location within reach (see keras package org. for inspo)

- Organized around some canonical examples


Dynamic vs static graph?

https://www.geeksforgeeks.org/deep-learning/dynamic-vs-static-computational-graphs-pytorch-and-tensorflow/

I think we should go for static graphs...

I think it will make the distributino simpler? We are not really changing this sytem that much? idk..


Ok so one view of it is the dataflow view:
https://dora-rs.ai/python/dora/builder.html

Thats what we are doing now... I think the cool thing is that you can inject just normal python nodes though and then you can use the function like normal?