

Q: Do I want to update mimic joints in my code base? or use the pinnochio parser for mimic joints?
A: It would definetly make sense to avoid having to implement yet more code in this code base if you have it already in pinocchio

Q: Should I make an abstract robot model or have a definite depency on pinnochio?
A: I think its fine to depend directly on pinnochio, it seems fairly fully featured. Alternatives could be KDL, Drake, etc. but I have enjoyed using pinocchio so far

Q: What type of install should I do?
A: Well if you need the mimic joints then you need a newer one. Pip seems to provide that but expects newer numpy..

- RobotPkg install seems older 3.4.0 doesn't have the thing I need. It may be beacuse I am using an older python version... though?
- Let me try the pip version.. then


Q: What to do about the viz?
A: Well this should really be very very light weight. The majority of the viz should go inside the bam_artist package, which lets you draw onto anything! I do like these pin utils being very very light. I do think of course its helpful though to be able to quickly bring up the urdf that you made...


Q: Should I have a srdf with links in order to test the collision? Yes but actually I can even just precompute one!
