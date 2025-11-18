
Idea is that you can easily import any msg type from

bam.msgs 

This makes it super easy! and fast when prototyping. For more verbosity you can always more directly import the files, and if file loading speed becomes a problem then you can comment out the top level import, so it won't load until you access the lower level module.


how to implement space for msg?

Msgs are very spefici obejcts, they refer to data structs that flow within the graph between components

Is robot_description a msg? is link_descripton a message? No it doesn't flow!!! Well i guess techcnially it could. I guess tehcnically you are right, but thats not the design intent.

