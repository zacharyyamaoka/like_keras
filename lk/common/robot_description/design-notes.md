Inspired by: https://github.com/david-dorf/pyppet

I want to take another go at simplifying the robot description

What do I like about xml?

-> Ability to see it all listed out

-> Ability to quickly prototype, make adjustments to lengths, etc, and visualize with tooling (ie. vscode urdf viz, etc.)

-> Standard format that is accepted into many different softwares (Pinnochio, rviz, etc.) its a great protocol.

-> for mature hardware, a static file is ok, and most hardware is actually quite mature!

-> Fast and simple to get started!

What do I not like about it? 

- Making it conditional, etc. is a mess
- Hard to combine and extend
- You end of generating all of these files
- See UR example of manging family of robots, they did a huge refactor to make it work
- Not type checking, syntax highlighting
- Hard to "hack"

I have been moving away from these static configurations files into just using python for everything

Follows a general trend of launch files in python, etc..

Sometimes you just want to quickly make a mujoco model with xml... sometimes its a pain to have to use the python. I have been there before!

Even if I don't support the full RobotDescription to xml right now... I already am starting to explore that area with the composite description and loading the configuration files...

So that is certaintly one pattern

- Dump your self to a config file
- Wrap a macro in a config file <xacro:make_config> that accepts config file, and unpack variables inside with python dictionary
- Optionally expand in isolated environment

I like the idea of having the one description that I can extend with any other... its a unified approach..

---

Here are the small changes I want to try next

lock_joints should not be part of the description I think? I feel that is osmeting you do to the description

I like the idea of being able to tag the links with a thing say world, or base_link.. which makes it easy to look up via tags.

organizing the informatino all under joints and links.

What about all the other things? even the joints should be types you can use thorugh the robot thing no? yes likely...

what about having multiple geometries per Link? makes sense right...?

One thing I am realising is that people are going to have different requirements... when you make something for everyone its different. I personally like to overload my descriptions with lots of data! mabye thats not the right thing though...?





---



    Base Class for all robot descriptions.

    Design Notes:

- We assume all robots are described in a single URDF/SRDF
- Using dataclasses for clarity. We have considered yaml, python classes, .py files, etc.
- We want to have a single source of ground truth. Avoid duplication of information, as then you need to update in multiple places.
- There are many file formats, yaml, urdf, etc. If we define it once here in python, we can autogenerate all the other formats
- We can also easily use "macros" to generate variants for different arm lengths, etc.
- The robot description should not easily get outdated (so should have minimal dependencies). It should just list information instead of implementing functions, etc.
- Decision to make this python file the base ground truth. Then you have a URDF which can hold the information regarding the joints/links/etc. 

    and can be parameterized via xacro args, that this python file holds.

- Make everything just reduce down to dataclasses, then I can easily save an entire complicated descirption as json.
- You don't need to rewrite all the <
- This should work completely without ROS!

    References:

- Greatly inspired by using python to import relevant params: https://github.com/AndrejOrsula/pymoveit2/blob/master/pymoveit2/robots/ur.py
- UR great reference for all the parameters required for robot and also manging a family of robots: https://github.com/UniversalRobots/ur_description/tree/rolling/config

    Q: How many yaml files to split the description into?

        References:

        UR uses 4: 

            joint_limits_parameters_file

            kinematics_parameters_file

            physical_parameters_file

            visual_parameters_file

        Moveit Uses:

            joint_limits.yaml

            kinematics.yaml

            inital_positions.yaml

            etc...

        v1.

        I think some amount of namespacing is helpful, I will just combine the UR and Moveit files...

- joint_limits.yaml
- inital_state.yaml
- calibration_config.yaml
- physical_parameters.yaml
- visual_parameters.yaml

        v2.

        Python file is now single source of truth. No need for holding yaml configs, like UR does. Its still nice to have YAML to load in

        params into the xacro though. We can autogenerate a a single config.yaml file which holds everything. This is a good pattern beacuse

        as you change the config names, etc and pass in more info, you don't need to keep on updating the functional delceration in the xacro file.



---

