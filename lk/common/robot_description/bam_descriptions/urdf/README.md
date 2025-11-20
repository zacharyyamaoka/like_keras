





Q: What should I do now that there are not going to be ros packages by default, I still need xacro to be able to resolve the packages...

A: I do like being able to visualize the packages within vscode... the issue is that the parsers don't work that well. Cannot run python code... well ok I have worked around that by having a version that doesn't require any python code.

I guess I can do the same now for the package resolve. A issue is that there are no longer and ROS packages for it to find. which I think is ok, it just means I use absolute paths now. I don't really want to use absolute paths in the file though as it changes... ok what options do I have? Well you can set an env variable or you can use an arg?

Env variable seems to make the most sense, but that doesn't work with the vscode... ok.


<!-- <xacro:include filename="$(find bam_descriptions)/urdf/common.xacro" /> -->
<!-- if include this here, there may be errors if included agian -->
    
<!-- When defining the joints, do as few rotations as possible, so its easy to orient back to base_link -->
<!-- Its better to not do any visual rotations, then the CAD values can be read directly, otherwise you may need to switch axises -->

<!-- 


URDF is designed to be self contained and mimic a UR robot
- If you don't shift from the base_link_1, then moveit should work for right/left arms
- Put all axis of rotation along +z (makes it easy to visually see)
- Convention for mesh export is to have z+ through the body, and y+ along forward direction/link
- Due to differences in UR and BAM construction, you may need to flip z (like for axis 1)
- By making it as close as possible to a UR robot, it makes life easier 

IK Goes from

base_link_1 -> ee_link

ee_link is a special frame used for IK tip in ur_kinematics

You can mount the eef to the eef_frame which has z going outwards, as expected... TF transform can be used to go from tcp_world/tool to ee_link

 -->

 <!-- Notes on bool vs string https://github.com/ros/xacro/issues/145 -->
 <!-- Using "true"/"false" seemed buggy as sometimes its bool/str instead use config and pass a string -->