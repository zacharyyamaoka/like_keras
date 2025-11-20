Thinking in like-keras. 

- descirbe our systems with the following terminology.
- Component Based architecture like React and Keras

>  
>
> https://react.dev/learn/thinking-in-react
>
> React can change how you think about the designs you look at and the apps you build. When you build a user interface with React, you will first break it apart into pieces called components. Then, you will describe the different visual states for each of your components. Finally, you will connect your components together so that the data flows through them

https://dora-rs.ai/dora/overview.html

https://docs.ros.org/en/iron/Concepts.html



- We will represent the entire system as data flowing through a cylical graph with the following types:
- Component is the atomic unit in the graph
    - Each Component has:
        - Inputs/Output Ports
        - State
        - Diagonistics
        - Logger
        - Sub Components
        - Configuration (static dict based struct, writtable to human readable file, that can reconstruct the component completly given the class, class + config = component)
        - Documentation (.md)
        - Visual layout properties
        - Unit Tests
        - Examples
        - Metadata
    - Components are base the unit of computation in a lk graph; each node should do one logical thing (SOLID - Single Responsability)
- Nodes are execution containers for Components
    - Nodes can communicate with other nodes within the same process, in a different process, or on a different machine
    - Nodes can be implemented in different languagues (Python, C++, Rust, etc.)
    - Nodes are what allow us to deal with the async nature of the system (wouldn't be needed if just directed acylical graph (DAG))
    - They don't describe any computation them selves
- Systems are Containers for:
    - Components
    - Nodes
    - Sub Systems
    - They don't describe any computation them selvs
    - They can declare "public api" input/oputs, which connect to the components/subsystems inside, which allows them to be called just like a component



- Then we have all these other helpers (compilers, datarecorders/playback, loggers, etc.)
- The system provides common template systems for robots (gym agent, SPA agent, pipeline_agent, multi-agent) at different levels of resolultion. You can configure these if you want or making it completley unique
- The system provides common components for robots, which we group into the top level mdp categories of (sensors, world_model, actor, controllers, actuators)
- RobotDescription is the description language for the hardware
    - They live at the level of the system and a given system can only have one robotdescription
    - As you create systems with subsystems, each of thoose subsystems will still hold onto their own robotdescription, but they will all be combined into the top level system.
    - The robot description describes physical parameters of the hardware sensors and actuators components, and how they are connected (via links/joints) 

Process for system design:

- Inspired by: [4+1 views](https://en.wikipedia.org/wiki/4%2B1_architectural_view_model) and [C4 model](https://en.wikipedia.org/wiki/C4_model)
- First start with the bigger picture, what are you trying to achieve, who are the stakeholders/customers? what is the conops? Can you describe a story?
- Start with a story. Explain how to achieve what you want in terms of components and dataflows. Start with the simplest product that is useful, commerically viable.
- Its helpful to start thinking about he system in term of components, what are the actual computations that need to be done, and dataflow, and what are the messages that get passed between them. What are steps in which that data is passed and the components are "activated"
- Keep on adding components/sub components until you can accomplish the desired functionality
- The first system you make should be the first thing that is usable together. A motor (muscles) and computer (brain) and linkages (skeleton) I would consider components, each of which have many subcomponents, but I still would not consider them a system as they cannot "work" without each other. A system is about bringing together many things and adding functionality relative to deployment/operations like .launch() etc.
- Now when you actually go to deploy you need to think about nodes. How are we going to group together these components?
    - If they are sync, sequential, same language, likely its best to keep them all in one node
    - If they are async, parrallel, different language, shared remote resource, likely its best to seperate into different nodes
- Now we assign thoose nodes to actual hardware where it will run.



like_keras should really be considered as a description language!

- Describe what the system is and how it works, then go make it! 
- I find once you describe with sufficient clarity the why and then the what, the how can follow naturally (wethers thats a 3rd party library, ai, open source, etc..)
- AT the heart of like_keras is a goal to allow us to describes these complex systems with sufficent clarity, and viz tools to help us understand the flow, and the performance.
- The only full description of the system is basically the full system itself, everything else is an approximation. We want to be able to seamless go from the highest level, down to the lowest.
- The idea is to try focus on the core of the system at first, in the simplest possible way. What are the components and their connections? And leave much of the complexity for later!
- Now is the fun part. To implement each of these components. Time to take out your favorite ide (cursor atm) and start programming!
- Goal is to be able to describe the entire system... even the dataflows that go in the cloud, etc. as its all related...



- If we don't assign nodes then they will all run in main.. or if different languages will complain... actually we just support python for system design, the `__call__()` is used to both construct graph in pythonic way and also for sync execution.



A special type of component is the behaviour tree.

- It has many sub components which it connects with a unique type of [control flow nodes ](https://robohub.org/introduction-to-behavior-trees/)(Sequence, Fallback or Parrallel)



Similarties to Keras:

- Component -> Layer
- SubComponent -> Layer within a layer
- System -> Module
- Sub System -> A modulue within a modulue

