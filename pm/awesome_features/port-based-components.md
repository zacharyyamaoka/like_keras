

Truley modular components


A huge difference we we no longer define exact points within each component, but instead just a port that you will wire later.

Trades offs, but here is what that work flow can look like for a 4 arm robot

Basically just set up the workflow for 1 arm robot, and then copy paste the code 4x. No need to worry about prefixes, etc.. (not true actually you will need to prefix the components so they are unique within the node) But you can see the power. We can be moving around these really large pieces of code...


work flow for 1 arm. Turn it into a system... that we can test indepently launch etc..

Make a rack system by combing 4 x thoose systems

make a robot row system by combing 3 x thoose sytems with a conveyor component

make a stack/mrf in the box, etc..

We can describe the interconnections of an entirly distirbuted complex multiagent system all in python, all with a single config, all programitcally generated... Its such a large system your literatlly generating the defintions with four loops, very powerful...

We compile the entire system, and its running accross who knows how many computers, how many GUIs. But its all just dataflow, and all the neat tricks and tools that work for simple systems will work for this huge systems...

A key feature is the python based descriptions that allow for this easy scaling and combiation... :) 

For these systems we want classes, beacuse we want the explicit type hints I belive in the intput outputs...
