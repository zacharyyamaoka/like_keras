

Abstract away language.

Select the one that will give you the best run time performance

Workflow is to generally implement first in python

- Test that behaviour is as intended in python
- Write unit tests in python

- Send a prompt to an ai to reimplement the system in the selected language. Right now we just support cpp
- You never really do any viz at this level, its just a drop in replacement at run time basically

How to verify that the implemtnation is correct?

We want to feed in a bunch of value into both components and make sure that the outputs come out the same

- We may want data to be time series in case that matters
- We may want the data to be from an actualy dataset distrbution..

- Msgs support defintinos of their data distrbution via `data_dist`
- the simplest datadist is like a uniform box with min max, by default there is no dist.this will throw an error


- Define the distributino for the input (unfirom, pareto, etc.) and sample it
- Iterate through a real data set (feeding in the datapoints in time series, and potetially at real time, or with rtf)
- That real deteset can be generated randomly or values can be in time series, etc..

- Ok it seems the end point is a dataset of input/output values

- Would be nice to have histogram to view the different values (they could be dataclasses though btw)

Then if its a pure function, you can just test its inputs and outputs



```python



```

What will this require?

- A method of comparing two text files with the outputs of the distribution (ground truth could be text file, or actually better I think another dataset.)

- I like text files because it becomes easy to instrospect. Easy to use diff/diff. And you can compare various differences at the same time, to get a better idea of what is eactly going on.

- We can then view the differences in a number of ways. one is text diff diff, but perhaps graphical analysis will be better suited.


- original component/system (You can mock between any inputs and outputs basically)
- A util prompting function that takes in the orginal component, a target language, and outputs a prompt that you can copy paste into any code editor to generate the new code (likely cursor agentic, beacuse it can set up everything) (key things it can set up are the acceptance test, doc explantions regarding the api of the new language, etc.)

- The new compiled comopnents which does the mapping between thoose inputs/outputs in your desired programming langauage

- a dataset of input output values. May be randomly organized, may be ordered. May come from randomly sampling (n_samples = 10000) the msg.data_dist or from historical data. May come from combing different histocral datasets. So we need a dataset object and we also need to add a DataDistMixin or just subclass it... to the base msg...

- A way to Feed in the inputs into the new component and store the outputs into another dataset. Option to feed in component at realtime if they are timestamped (helpful if time ordering matters) or with rtf (real time factor to just go as fast as possible). This seems like a generally helpful feature. Essential its like a rosbag playback ( this can be used more generally, alot of the funcitons in here can be really..)

- For sensors, its more challenging. If you can implement the driver as a component then you can mock the next level of input output.
- At some point though then you can manually mock the sensor input say, Put a paper of a known color infron tof the camera, and take an image, or look at a static scene, running both at the same time? Ok love that yes... snaps... actually the sensor may not support that. highly likely the driver does not support reading into two different things at the same time. So you can run seperatly. If a gazebo simulator etc then easy to just replay the exact same world...

- Compare the datasets. If its like a table, you can match the keys and then compare the outputs. First verify that the keys match, and the relative tick count or relative time is the same (abs time may be different... with flags you can perhaps set the acceptance criteria)
- Then compare the values...

- Output diff diffs to a text file for analysis
- Output histogram of both distrbutions (they should be overlapping perfectly or you can see the difference)
- Output heatmap of the to see differences etc
- Output image to see which pixels are wrong, etc.




---- Further notes from the prompt clarification
1)

for Ai compile I am imaging the following structures

create_prompt() (this support also support a component). actually utlimatly I think the prompt should be about inputs and out specifications.. perhaps if we recorded some data se can also attach it?

(key feature for create promopt is to autoically get relvant context, like the relfvant api for the target languague, etc.)
prompt could be for a component, a system or just a random set of inputs and ouputs. (or essentiall just ports.. the input to your program can be the input from one or output of another, its not a directed acylical graph)

send_prompt()
these can be utility (lk.utils) functions that accept a component. they feel a bit seperate which is why I think best to keep seperate
the idea is that you keep around your create_prompt() so even if you update your code you can just rerun the compile script agian and it will "recompile" the pthon code to what ever language you want.

I think we want  prompt dataclass that stores all the data as needed...
(can live in lk.utils) msgs are very specific things they refer to the things that flow witin the graph

We then need a Dataset Struct I feel this can live in Common

I have no implmented som eof the DataDist stuff you can check it uot in messages: @data_dist.py 
@pose_stamped.py 

So now we can create a dataset by smapling a DataDist or by using real data

- We then need a Playback class

this can live in Common. It should take in a dataset and then play it back in seqeutnail order, with optional RTF control.

I think we probably want ways to read/write that dataset:

see foxglobe sdk:
https://docs.foxglove.dev/docs/getting-started/python#recording-data-to-a-file 

actually the dataset is a really big topic. 

there should be af older in common for it beacuse there is gonna be alot!

When we think about datasets...

I think we have the python API for the dataset... but behind the scenes...

could be alot

so we should have a generic ABC for dataset just like most of the common compnents should have.

what does it mean to save the "data" 

What is the "format" of the data mcap or something else?

Do we save with a ring buffer, etc, do we do live continous upload to S3?

writing and uploading may happen in another component for example async to avoid blocking...

so deifntly I think these various dataset actions we want to keep generic so we can change their implemtnations

Once you have the dataset.. 

then we want to support various viz functions. For now you can just use stubs.

plot_histogram is a ncie one, 
also diffdiff(other_dataset) is an important it compares the two files and creates a diff diff git hut like page that should the different entries in the dataset

We now need this way of feeding the inputs back into the new component... I think you could do this in an silated inevionrment, but it feels like you can also just leverage your existing stack and in the simplest case make a 2 component system with teh playback compnent and the 

potetially another dataset recorder to see the result and the actual coponent you are mocking... Its nice having simple things you can use in cool ways...

For the dataset I think we can really leverage alot of the ideas from Rosbag:

https://github.com/ros2/rosbag2 

It seem we hav a Recorder and a Player

We want to use a back end actually I think to do this wether reductstore, macp writer etc..


foxglove for example seems to have a very simple api to load to save data: https://docs.foxglove.dev/docs/getting-started/python#recording-data-to-a-file 

or reduct store sems cool as well:

import time
import asyncio
from reduct import Client, Bucket

async def main():
    client = Client('http://127.0.0.1:8383')
    bucket: Bucket = await client.create_bucket("my-bucket", exist_ok=True)

    ts = time.time_ns() / 1000
    await bucket.write("entry-1", b"Hey!!", ts)
    async with bucket.read("entry-1", ts) as record:
        data = await record.read_all()
        print(data)

loop = asyncio.get_event_loop()
loop.run_until_complete(main())

here is good implementation of reduce store in python:

https://github.com/reductstore/reductstore_agent/tree/main/reductstore_agent 

Ok imporrtant 

-- here are not on the bag playback
In ROS 2, ros2 bag play is basically a time-controlled message replayer: it reads recorded messages from storage and republishes them on the same topics, with (almost) the same timing they were originally recorded with.

I’ll break it down in layers:

1. What a ROS 2 bag actually is

When you do:

ros2 bag record -a   # or specific topics


ROS 2:

Subscribes to the selected topics.

Stores each message into a storage backend (default: SQLite DB, via rosbag2_storage).

Records metadata (topics, types, timestamps, QoS profiles, etc.).

So a bag is really:

A metadata YAML file (metadata.yaml)

One or more storage files (*.db3 by default), which contain serialized messages + timestamps.

2. What ros2 bag play does conceptually

When you run:

ros2 bag play my_bag


the play node:

Reads metadata
It loads metadata.yaml to know:

Which topics exist

Their message types

The recorded QoS profiles

The recorded timestamps for each message

Creates publishers
For each topic from the bag, it creates a publisher in the running ROS graph, using:

The recorded message type

A QoS profile (by default, something derived from the recorded QoS or a safe default)

Reconstructs time
It defines a “bag time” that starts from zero at the first message stamp.
Then it:

Calculates the time difference between messages (Δt) based on recorded timestamps.

Uses a timer loop to sleep Δt between messages (scaled by --rate if you want faster/slower playback).

Publishes messages
Each stored serialized message is deserialized and published on the corresponding topic at the right simulated time.

So from the rest of your system’s perspective, it looks like a regular node is publishing those messages live — just coming from storage instead of hardware or a live algorithm.

3. Controls / options that matter

Key flags you’ll actually care about:

ros2 bag play my_bag \
  --rate 2.0          # 2x faster
  --start-offset 5.0  # skip the first 5 seconds
  --loop              # loop forever
  --topics /camera/image /tf  # only play some topics
  --remap /old:=/new  # remap topic names


Common ones:

--rate
Scales Δt between messages. --rate 0.5 = half speed, 2.0 = double speed.

--loop
When it reaches the end, it jumps back to the beginning and keeps going.

--start-offset
Skip some initial portion of the bag.

--topics
Only publish a subset (useful when you recorded “everything” but only want some topics now).

--clock (or use_sim_time)
If you enable /clock and set use_sim_time on nodes, you can drive the simulation “time” from the bag, so things like TF and time-aware nodes stay consistent.

4. How timing actually works vs. ROS 1
ROS 1 (rosbag play)

ROS 1 had rosbag play, also reading from a bag file and republishing.

Very similar idea: timestamps, playback, rate scaling.

Time is handled via /use_sim_time + /clock if you want nodes to follow bag time.

ROS 2 (ros2 bag play)

Under the hood, playback is built on rosbag2, which is more modular:

Storage plugin (SQLite, MCAP, etc.)

Serialization plugin

Converter plugin (for format/type migrations)

QoS is much more explicit in ROS 2:

The recorded QoS profiles matter a lot. If subscribers’ QoS doesn’t match, they might not get messages.

Often you use “Best Effort, Volatile” or similar to make playback more forgiving.

But high level: ROS 2 playback semantics are very similar to ROS 1, just with extra complexity around QoS and the plugin architecture.

5. How your nodes see playback

From any node’s perspective, bag playback is indistinguishable from live data:

They just see messages being published on topics.

Header stamps are whatever was recorded (often original sensor time).

If you also play /clock and set use_sim_time, rclcpp::Clock / rclpy.get_clock() will advance with bag time instead of wall time.

So you can:

Replay sensor data through your whole stack.

Test new algorithms on old recordings.

Reproduce bugs by replaying a problem bag.

6. TL;DR in one sentence

ros2 bag play spins up a node that reads serialized messages + timestamps from a storage backend and republishes them onto the ROS 2 graph with the same topic names, types, QoS, and relative timing as when they were recorded (optionally scaled, looped, or filtered).

Interesting...

If we make a bag that can dynamically connect to any of the ports,,, no need to define ahead of time. perhaps we can dynamically conect during playback?

when recording

we have a recorder(in1, in2, in3 ....)

during playback

out1, ou2, ou3 = playback()

If we unpack something that is not there then it can throw an error type thing... this makes it very easy to route the thing I think... following the same api... which is cool :)
read/writing from memory is just like messaging passing but through time - Claude Shannon (what  a boss)

for 2) Lets star with basic structures for all these ideas I want to start with a coarse view and slowly hone in

3. for dataset storage format, we should be indifferent easy to change later! 

4. The datarecorder should be a component the dataPlayback should be a component

they should use this api like the rest of the components

Please also add a summary at the end of this document about all these ideas we are trying to implement



----