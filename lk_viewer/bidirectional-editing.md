


The idea was to have a web based viz, that could run locally

- Similar to:
    - Roboflow inference which locally can run the workflow editor
    - Foxglove
    - Moveit Pro
    - etc..
    - Intrinsic Flow seems to be able to completely run on the cloud...
    - Viser

- Generally I just think this is best practice today for usability, ease of use, etc.. I like how I can have the webserver kinda running seperate to the 
Technology Stack:

✅ Web browser viz (like foxglove, roboflow workflow, viser) 


Q: How are we going to draw the graph?

https://reactflow.dev/

✅ React Flow



Alternatives:

    - It seems that pytorch and kera use pydot/graphviz: https://pypi.org/project/pydot/
    - https://py-trees.readthedocs.io/en/devel/visualisation.html (using pydot)
    - A more modern library is mermaid. still mostly static 
    - https://mermaid.js.org/

    - Behaviour trees seem to have a js implementation or qt if you want it local: https://visjs.org/
    - https://github.com/splintered-reality/py_trees_js
    - but seems now using d3? https://stackoverflow.com/questions/28595732/vis-js-how-to-expand-collapse-nodes-with-mouse-click

    - This seemed like alot of work... then I found this, which is great its a python backend for a web viz, basically what I want without the effort.
    - https://dash.plotly.com/cytoscape : this one is super simple and like Viser provides a nice python API
    - It seems that Dash uses flask: https://www.reddit.com/r/learnpython/comments/pauixb/what_should_i_learn_dash_or_flask_interested_in/

    - Then chat found react flow, I was wondering what inference uses...


"Back End":

- FastAPI
    - Used by Roboflow which is very similar to what we are trying to do
    - https://inference.roboflow.com/understand/alternatives/
    - Python based so we can actually almost directly reuse our python objects, no need to implement new objects.
    - Certain having a python back end is a huge benefit beacuse we save almost an entire read/compile cycle if the end/points can directly call 



Key Goals:

- We want to seamlessly be able to switch between different views of the data, view/edit the data in different views and continue editing in another view.

- If I have to state the problem generally.

- Or concretely

We have a system that is composed of 
- `components`
- `node` 
- subsystems
- subcomponents

This mirros keras
- component -> layer
- system -> model

and just like in keras the system behaves just like as special component just with some special features, so everything is really compnents.

However we build it, at the end of the day we will have a graph. Composed of nodes and edges and sub graphs. Each of thoose nodes and edges may have unique properties.

For round trip what is required?



1. Is this idea of having a Internal representation, a single ground truth, that all systems and edit and display from. If you want to go round trip than that groud truth should be a more a CST (concrete semenatic tree) vs AST so you can represent things like comments, in the graph, and mabye things like layout for the visual view.


```
def generate_system_functional_simple(system_config: Optional[System.Config] = None) -> System:
    """
        Simple functional approach with optional config.
        
        Good for quick prototyping and simple systems.
    """
    # First define nodes
    node = Node(name="agent_env_loop")
    
    # Define Components and assign them to nodes
    agent = Agent.from_node(node)
    env = Env.from_node(node)
    
    # Define Connections between components
    # The callable interface creates connections automatically
    action = agent(env.obs)  # agent receives env.obs, returns action port
    obs = env(action)        # env receives action, returns obs port
    
    # Create a system
    # inputs/outputs become "public APIs" for introspection
    return System(
        nodes=[node], 
        components=[agent, env], 
        inputs=[], 
        outputs=[action, obs], 
        subsystems=[], 
        config=system_config
    )
```

Q: How to update the python code after updating in the graph?

A: This is actually a very complicated problem!!  

What if you add new components in the graph? What if you make new connections?

This will update the internal representation, so the python code also needs to be updated to match that.



Optiona 1 could be you cannot add new compnents you can only configure ones that are added in python



---


Ok

so I want to start very simple with this lk_viewer

at the core I want bidirection editing and reactive viewing

There are so many features that can be done, but let me just start simple.

lets describe an open ai gym basically.

How can it first display it?

if I have a python desciption of a system, then I can do .compile or something build up the static graph and then its easy to do. If I have a bunch of random components floating around, then you need to build a full parser. This is not ideal..

perhaps if you have everything on one page though or one directory it can help?

lets say for urdf dev.

I have a file for a description where I am defining things...

hmm actually you have a robot description class.

I should be able to start a server that links to that robot description class and visualizes it, and I make edits in python it should edit..

as you update the python code, likely want to switch between put params directly in class vs in a config.. I think here its interesting where for the URDF its fairly hard and requires alot of code to add the mesh, etc.. but to be able to view it, move the joint angles, adjust the link lengths, this should be simpler..

the question is how do I then reflect that databack to the python code?

It wil be a simlar challenge for the system... the robot description is just one part of thata...

at some point like instric you may want to be able to add new links to the scene, and then add fixed joints relative to the world? I guess is essential what they are doing or other links? for now a first step I think is just viewing the pytho config interactively..

let me make a simple example of a rr robot description. 
Here for example is a robt description

@bam_fb_dev.py 

I see two optoins. 1 is that it tries to intellgiently persve the formating. but what happsn when you add new items?

2. is that it compeltly regenerates accordingly to a schema

3. is a blend between the two

I think to start I can just do 2.

For me its like teleoeprating and arm master and slave style.

When you are making edits in python, the ir yaml updates and you send the info to display reactive in the ui. If you edit the ui, then the yaml updates and now the python code updates..

v1 is the python code will completely regenerate. Perhaps the yaml can optionally have information regarding the class that made it? and its file location, then you can just update that file. but what if other info on there? you don't want to accidently override..

perhaps the best is if you always start with the graphical editor baically autogenerating the file structure? then lets say you want to go in there and hack around... hmm mabye its ok mabye I just update things associate with class?

Let me know there are some trade offs here.. how should I proceed


principles:
1. Make software for ourselves. If it happens to be helpful for others great, but we want to optimise for solving the problem at hand in the simplest, fastest way, and this is a tool to achieve that. It's purely functional.
