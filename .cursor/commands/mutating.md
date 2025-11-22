


Each Component and System shold get their own folder.
- This is important beacause we want to track alot of information regarding that component near it
- perhaps it will be simpler if we keep all the code colacted? for simplicit?


- Each component/system should get their own folder
- There is alot of information we want to organize around it


/{component_name}
/tests - Units tests
diagonistics.py
tests.py
v1.md # we can also iterate on the prompts and documentation as well
v1.py
v2.py
v3.py
...


- Organizing as a folder makes it easy to get context as isolated
- The layout then matches the layout of the software



Types of mutation

Mutation - which is editing a single program to make a new one

Cross Over - which is when you combine X programs to make a new one

# all the different iterations



https://en.wikipedia.org/wiki/Genetic_algorithm#:~:text=of%20the%20above-,The%20building%20block%20hypothesis,-%5Bedit%5D


We can mutate at various different level. One is at the line level, one is at the component level, one is at the system level.

- We can control the amount of cross over and mutation by controlling # of lines changed. This is like the learning rate in backprop.


All the code in the software is the geonme that you can then mtuate


at th start of the design I am right there with the ai, hand holding it adjusting the circulaum very specifically. make small concrete steps so that it can try to improve. I think as you build up though you can give it more and more space. Avoid the temptation to basicaly specificy any implementation which is what I am completely give up on now and feels great honestly...