One issue with a RL like metric is that the feedback loop is long. It takes a bit of time to see if its working or not

- What can we do as humans? We need to let it run for some amount of time, ideally start with a smaller problem just to verify optimzer is working
- See [Andrej's notes on this](https://karpathy.github.io/2019/04/25/recipe/)
- Generally though starting with toy problems (Cart Pole)
- Sutton's "approximation the solution not the problem"
    - Cartpole is still the MDP problem, a 1 layer neural network is an approx solution
    - Supervised learning is a different problem, etc.
- I think you can do summarise statistics, but the idea is to hopefully allow the agent to see the high order visual patterns. without creating an entire datasytem that predicts if its increasing or not..
- https://briefedbydata.substack.com/p/same-mean-different-distribution
- Here are some great datadriven approaches:
- https://www.improvingagents.com/blog/best-input-data-format-for-llms/
- I am concern with aboslute accuracy here not cost

---

Prompt.

Please make a python script that generates various different types of potetial RL learning learning curves. I think we can simulate the behiavour by creating a simple multi armed bandit problem. 

Select the number of arms. Randomly intalize the agent, randomly select its epsilon value. then just average the scores and with espilon greedy select arg maxs

To generates a diverse range of curves

- Lets plot the reward for 1000 time steps
- For each experiment select random variables within range for
    - num. arms
    - mean and std of the arms (this will lead to nosier or smoother curves)
    - agent episolon value

To test its understanding for early stopping.

- Please ask it to look at the data and predict the point at which it would be helpful to do early stoping.
- Ask it to rank the graphs in terms of which had the fastest rate of learning
- Where is the inflection point at which the thing started learning?
- Ok basically I want to make a simple dataset..
- Rank the graphs in the following order
- Which had the maximum performance?
- Which had the fastest rate of learning?
- Which ones had the slowest rate of learning?
- If you were iterating, which graphs would you have stopped early? (this is like a great supervised learning problem)
- Of course having seen more than one graph its clearer to me. The fair comparison of course would be to show me each graph in sequence and then ask me if we should continue testing or not... Well I need to get a rough idea about what the performance is to determine that, thats why having context is important... perhaps thats why having the baseline is important?
- Categories are hard I think...
- mabye for v1 you can do a completely random agent that doesn't learn so I can get an idea of baseline performancef
- I like how you take the view of the agent here...

This is great beacuse its super simple but captures alot about the RL problem

Interesting - potetially thats like a genetic algortihim? Or you can take a conesus approach? I feel that perhaps an algorithim could almost do better than an agent here?

Why not combine both!

- https://stable-baselines3.readthedocs.io/en/master/guide/rl_tips.html
- ok this is basically hyper parameter tunning algorithims: https://araffin.github.io/post/hyperparam-tuning/
- https://araffin.github.io/post/optuna/

Ok bingo. its the field of hyper parameter tunning. How do you early stop things which are clearly bad in an optimal way?

- This is gradient free design. Exactly what we are doing here!!!
- Perfect this is applied to RL: https://araffin.github.io/post/hyperparam-tuning/



![alt text](hyperparam-tunning.png)



* Wow super cool :)
Over time, you can also let the LLM propose configs conditioned on past winners (“copy best but tweak LR, increase model size, etc.”), but that’s a bonus layer.
* Each commit could be a thing, and then we can revert to that branch..
* Reward can be any metric but its just a single scalar value to track.
- Background agents? Cloud Agents? Yes if they have a shared database... 

Could be cool to connect straight to weights ana baisses I can track how they are all progressing :)


- Weights and biases for tracking experiment progression... defintly yes!!

- These seems to be the ones to do:
- We use MedianPruner in most examples, though basically it is outperformed by SuccessiveHalvingPruner and HyperbandPruner as in this benchmark result.

- Ok just to help the agent iterate faster.. well for cart pole I think its simple... but for the other ones super helpful...

Ok how should this work?

Well the pruner should output text information... Well if we want it to be even more concrete

v1. the pruner can automatically stop. v2 the pruner could asked for ideas.

Still you are perhaps missing out on the big ideas here. So maby you still want the llm to look at the text dat I still feel thats powerful.


Ok so 

LLM sample (we do that already) assumes its essentially random search

LLM Prune

It should perhaps block and print out input for the agent to read...?

Optuna Prune


https://forum.cursor.com/t/how-to-automate-cursor-prompts-and-monitor-accept-reject-status-via-code/102744

Probably simplest will be to just directly integrated into the program no need to MCP, etc..


--- Alot of these pruners are for single variables, which is great for reward...

I think regardless it would be nice to essential give LLMs the ability to look at logs...

Lets test indistbrution! better than not in distbution...

We can make a little dataset if needed over time?

Lets say the metrics locally..

- We can just get humans to detect it...

- What would we want to say? natural language descriptions?... well we are not really training it, we just want to do incontext...

Lets the human see the full graph, and potetially in context with other graphs.

Pick two random graphs, and say which reward curve is better? or which loss curve is better we just flip it to make sure we show it both...

For the humans we can go thorugh reward curves and loss curves...

Basically we just want to give teh ai the ability to always compare perhaps vs some of the best ones to date, and the prune if it thinks its not going well...

LLM pruner...

- How many of the 

- The issue is that the summary statics could be misleading and what about the other metrics type things...

- Ok for now we can just start with optuna pruners tough...

- if they worked better than humans may work better than llms..



---

AutoML School

https://www.youtube.com/watch?v=I1H5nZhjsVo
