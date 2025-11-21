# Background

What is the simplest agent we could possibly make?

- It needs to be a system. 
- For the full modelling language/concepts please refer to [core-concepts.md](../../../docs/core-concepts.md)
- A system is the simplest thing that we can actually "launch"
- But actually all the compnents should also be able to work indepdently as well like python code. This is essentially like being able to compute with layers in pytorch, before you make a "model"

We have "templates", which are factories for different type of code we can completely reconfigure

Based on: https://gymnasium.farama.org/index.html

Basically this code should

- create cart pole agent
- create a cart pole env
- 1 describe this entire system in terms of components
- We should be able to bring up a graph in vs code to see the entire system, just like you press the button to view a urdf
    - https://github.com/MorningFrog/urdf-visualizer)
    - is similar to a markdown renderer: https://github.com/jonathanyeung/mark-sharp

      This

Its just a different "view" of the same underyling system.

- Christian Henkel has been particuarlly influencial on our views on diagonistics:
    - https://roscon.ros.org/2024/talks/How_is_my_robot_-_On_the_state_of_ROS_Diagnostics.pdf
    - https://github.com/ros/diagnostics

What does success look like? and how are we going to write a test to verify that it works?

How we know that the code works? If we achieve the target reward. For this we need the concept of Diagonistics (We abbreivate often to Dx)

### Diagonistics (Dx)

#### Intro

Diagonistics are a unified approach to testing that covers develop and deployment. They are quantitive metrics we use to verify and monitor performance.

for DRY (Don't Repeat your self) we want to just define what success looks like once, and ensure that its met throughout the life time of the product

- Diagonistics should be related to performance metrics/functional requirements
- If your code doesn't run, you likely don't need diagonistics, get a linter and fix it.
- If you can verify before deployment that all possible input/outputs pass then you don't need diagonistics. Its much more reliable to just make systems that "cannot fail". Diagonistics are for thoose things for which performance can degrade/improve over time.
- The art is in defining a couple of key metrics that provide clear view into the systems performance. Use them like highlighters, the more you highlight the less value each highlight has.
- Some metrics are pass/fail (these are like constraints) some are a range that you can continue to get better/worse at (these are like objectives you can optimize) By defining the correct Diagonistics, you enable the LLM to essential search the space of programs and improve on thoose metrics. Just like with a neural net, that you don't introspect fully the weights. We no longer need to understand as much as what is going on inside, if we can hold the LLM accountable to the inputs/outputs. A nice thing is that you can if you would ever need to look into the code (This is generally bad practice though, as the better solution is to "shape" the constraints/objects properly). And if you provide the right `.rules` then you can get code that looks like you wrote it.

Step 1. Define what we mean by performance.

Step 2. Define what metrics we can you to measure it

Step 3. Define Dx thresholds for Ok, Warning, Error.

So for cartpole. We have an agent that is going to be making actions, we will be getting obs, and reward.

How can we track that the agent is getting better? Episode length is getting longer, reward per episode is increasing, Avg. reward is increasing.

- [Great commentry on RL by Alex Irpan ](https://www.alexirpan.com/2018/02/14/rl-hard.html)
- Learning efficiency, how quickly does the agent improve? 

Common

The best agent in this case

- Achieves the high maximum score within the allocated training time

In this case:

Diagonistics/tests are really where the rubber meets the road. Everything else is just quite soft and ideas, absoutely essential, but to the extent it helps us better search that program space to find something that meets the diagonistics.

Ok so what are the diagnostics for this? A gool thing is that the top level diagonsitics basically never change if you are modelling the problem as an MDP, that doesn't mean you don't need to define more diagonistics though. Defining diagonistics at subcomponents/systems helps you better shape the optimization. and constrain it.

Essentially we are vibe coding this entire code base, but, to get a better result, we are breaking it up into smaller compnents with clear I/O and success metrics, that we can vibe code and then put together into a larger system.

Essentially what this means for us is that we just do the first half of the [V-model.](https://en.wikipedia.org/wiki/V-model) We design it, define the why and what, and then let the AI find the how.

> Ask for what you want, and then AI can deliver it for you

Component based architectures + diagonistics + AI is a match made in heaven. One issue I realised when trying to create a system that would let multiple agents work in parrallel, is that you need to define what things can be worked on in parralel. If their work is overlapping its very hard beacuse what your working on is changing. Components are great here, beacuse you are essentially defining the clear I/O boundaries. Even with a 1000 component architecture you can assign 1000 background agents to each component, or series of sub components, etc, You could almost rewrite the entire code base in a deterministic day each day. If you have kept your orginal prompts. You can do search over entire code base architectures. It all compounds on top of each other..

Optimize:

- length of episode 🔼
- reward per episode 🔼 (directly related to length so we only need to track one)

Constraints:

- None, to simple an environment

I think there are many ways though it could write code to achieve that? So its important to set up scaffolding



## Pruning

One challenge would be changing code with long feedback cycles. For example if it takes 1hour to test... I guess thoose are bad examples though. Code changes tend to be much more discrete, I guess that is why its not as important... Well think about the kaggle case actually. As you are iterating over ML algorirthims that can be quite a process.

How would you make an AI agent that could create a model to win a kaggle competition?

- There needs to be ieration within the model, and then iteration on the model it self
- LLMs allow us to do optimzation in text space. It can definetly do much better than random sampling, which is like the monkey and a type writer thought experiment. IT can even look at previous results, a performance summary, and incoperate the results accordigly, to plan its next phase. Really essentially this is what ML engineers are doing...
- The starting reference code for the agent is like a seed
- We can copy paste docs, new papers, ideas on approaches, other working models in order to improve.
- It would be nice if we can assign the llm to monitor certain datastreams, it would be nice if we can automatically prune on certain datastreams.
- Median pruner seems like a good start and nice beacuse I think you can do in a sequential way.
- 

#### Reference Code

# Prompt

The important thing usally to design is the API... Ok first first first, please make some proposals for API designs to implement this examples:

Ok first we need to make the tests we want the system to past.

- The simple approach would be a unit test
- The thing I actually really want to test though is the rl performance. RL is all that matters. the LLM could actually figure it out if it tries long enough...
- Ok so the first step I think is to see how we can easily get that RL metric.
- Each componenet has diagonistics, I think by default the system should get access to all of those 
- If we just run the system



- We need a top level diagonistic components I think, diagonistic aggregator that collects all the diagonistics from the children classes.
- then we can always query it eactly... Definetly a draw back of using the factory methods is you don't get the type hints, a benefit is you get flexibility. Depending on the resolution we can change. for templates It think use factory. each template will at some point use a concrete impemtnation though..



Ok I think we can have systen.pruner.monitor() # we should be able to then just pass in I think any variable, wether thats a diagonistics stream or a log, etc.

- This should be a value that increments as the system runs
- It should be a metric that reflects good system performance
- Can we potetially have many metrics? I guess but that is potetially 

Please make edits until /home/bam/bam_ws/src/like_keras/examples/basic/ex_01_cartpole/system.py passes its test

- One thing that is challenging here actually is the long feedback time with the RL environment. The nice thing we cart pole is that its decently fast, but I can see why this top level optimization may not be so good.
- Its important to be able to see early signs of progess and rate of improvement on metrics (signs of life) if you see it increasing then you can let it continue running, otherwise you can choose to stop it
    - Idea is we can make helper functions that output metrics in terms of text so the text based agents can better udnerstand









