
Right now making it as a ros utility, could be pure python, will see...

Ok I will make it a ROS utility, as the diagonistic msg type is a standard ROS message,
and that goes down into the core of this class where we track threshold values for warn and error.

## C$$$

I just made a custom tracker, when essential one already exists, grrrr. lol

What is similar between the library I made and the standard one?
- My goal was to seperate the calculation of the metric, and the checking of thresholds. The node has the context to calculate the metric.
- I create basic types instead of dictionaries for clarity/typecompletition/helper functions
- I started thinking about making helpers for commonly calculated metrics
- The concept of lazy execution
- The terminalogy of upper or lower bound
- The determination of the highest level of diagonistic or aggregate
- The registration of metrics with a parent tracker
- Easy conversion to DiagnosticArray()

What do I see in DiagnosticUpdater()?

- shared API between C++ and Python (as close as possible)
- Documentation and examples online
- Other people contributing changes
- Orginally design by an absolute G, ftoote,
- Off the shelf reporters for common metrics (RAM, CPU, etc.)


The `/diagonistic` topic takes type DiagonisticArray which has various DiagonisticStatus

- this library is built around DiagonisticStatusWrapper, which is a class used to create DiagonisticStatus. (these are like the metrics I registered)
- an Updater which is used to collect and publish (this is like my top level tracker)
- They pass in a node, to create the publisher, they then create the publisher, with len 1, which is helpful to see. Interesting that they then use a timer! instead of manually calling it...
I guess if you want to do it in a lazy fashion , decoupled from the actual computation... this could be useful if you have a node that doesn't get called much for example... Oh cool you could just manually call update... BOO YA! cool that we called it update as well... yup force_update() ok these guys are pros...
You can get the best of both worlds! set a long timer, and pause it, or force update, etc.

Well what can I say, it is a beautiful design I must agree. better than I would have thought my self :) certaintly more thorough and battle tested...
I like structure and constraints is how I move faster. Ok I am commiting to going with what they have. I think its very thorough.

Design Patterns:
https://ros.org/reps/rep-0107.html
- Use / in the name of the metric in order to add more specificity/grouping
- about 1hz is reasonable
- hardware_id: If applicable this identifies the specific hardware running. This is for things like serial numbers for devices so that a piece of hardware can be tracked between robots if it is moved between robots or moved within a robot
- message: A human readable summary of the status of the device. This is often a concatenation of any error messages or a default message.
- The system is generic enough to handle all software components, however adding diagnostics to all software components creates too much noise for the core functionality to work consistently. Setting up the analyzers to make sure that important messages are shown and there are not false positives of important errors is impossible for all users use cases
- Software cannot be changed by an operator, but hardware can be! I am not expecting the user to be looking at these, these are for devs.

https://docs.clearpathrobotics.com/docs/ros/cockpit/ros2_diagnostics/

- They aggregate and only show the top level diagonistic associate with an area, the way foxglove does it is ok as well
- Notice how they use namingspacing though

https://github.com/zulhafiz-zulkifli/ros2-diagnostics-example/blob/main/diagnostics_example/scripts/computer_monitor.py

Great repo with good examples

https://github.com/husarion/husarion_ugv_ros/tree/ros2/husarion_ugv_diagnostics
- thresholds can be set by yaml :)
- this is in CPPbut a great example of a production system. this entire database looks smooth asf

https://ros4hri.github.io/ros4hri-tutorials/interactive-social-robots/#:~:text=from%20diagnostic_msgs,KeyValue
- You can always just manually build the Diagonisticarray and publish it:
```python
def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/intent_extractor_chatbot",
            message="chatbot chatbot is running",
            values=[
                KeyValue(key="Module name", value="chatbot"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
                KeyValue(key="# requests since start",
                         value=str(self._nb_requests)),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)
```

## Best practices....
https://discourse.ros.org/t/how-do-you-monitor-your-robot-diagnostics-topic-rates/41191

Let me just try a couple different methods here.... Nope this is out of scope! do the simplest thing first!

https://foxglove.dev/blog/using-ros-diagnostics-in-robotics-operations


Here the best practices seem to be expalined:
https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/example.py

Ok A big benefit of using this package, is that it ports over to c++, and you get the design considerations already baked in.... 

I like there idea of making functions for common diagonistics...
I shouldn't really care how its done... tbh.. should I just use these? likely...

Is it right? is it wrong? Ok I am not sure but I think I will use it... everything you can do off the shelf, do it off the shelf!

Docs written for free... otherwise how will new team member know?

https://ros.org/reps/rep-0107.html
Using foxglove likely no need to aggregation as you can just filter by string and use /namespaces/
Hardware id should be unique to a phsical robot they are suggesting...

## TLDR:

Making diagonistics a first class citizen in testing.

At a high level all testing can be broken down into 

```python
def launch_descriptions()
# 1. Launching the system, were you define hardware, policy and env
system.launch.py hand:=crab_claw, arm:=ur, sku:=ur5 plugin:=gazebo, env:=PickBlind, policy=BlindPolicy

# 2. Launching the scenario
scenario.launch.py scenario:=simple

# 3. Checking for all greens lights:
def test_ur_pick_sim()

	diagonistic = DiagonisticClient() #this reads the diagonistics at a current point in time, and checks for any errors

	while True:
		rclpy.spin_once() #diagonisticlient makes its own subscription, you just need to spin
		if t > 10:
			break
		
	assert diagonistics.OK 
```

I think this will be incredibly effective beacuse the tests won't get autodated! They are checking the same things that the devs/operators would be checking
As you add any more diagonistics, they are automatically going to be checked as well. The idea to to define what "healthy" means once inside the node, and
then read the health status anywhere, from a test script or from a web dash board.

# FRs

FR1: Should be easy to add register new metrics and track them
``` python

    # set defaults to be used for all metrics, can be overridden on metric registration
    MetricTracker(
        name="loop_time",
        max_len=100, # can be overridden on metric registration
        min_len=10,
        mean_len=50,
        std_len=10,
        min_val=0,
        max_val=100,
        mean_val=50,
        std_val=10,
    )

    # Set good defaults so you don't you can have simple metric tracking, to full fledged metrics with thresholds and assertions
    register(
        name="loop_time",
        level=DEBUG or INFO # used to group into buckets for selective evaluation
        max_len=100,
        min_len=10,
        mean_len=50,
        std_len=10,
        target_val = 1.0
        mean_warn=1.5,
        mean_error=2.0,
        std_warn=1.0,
        std_error=2.0,
        min_warn=0.5,
        min_error=1.0,
        max_warn=2.0,
        max_error=3.0,
        lazy=True, # only calculate when requested
    )

    append(name="loop_time", val=1.0)

    status = tracker.evaluate("loop_time")  # → DiagnosticStatus

    tracker.to_diagnostic_array()  # → DiagnosticArray


        threshold:
            mean:
                warn:
                error: 
            std:
                warn:
                error:
            min :
                warn:
                error:
            max:
                warn:
                error

```
# Notes

## How did this class come to be?


    As I was thinking about a minimal set of tests for the bam_agent, I realized that from a high level there were KPIs I wanted to 
    track to make sure it was performing well, for example:

    Actions per min (#/min) - How many actions does it take in 60s, in most cases is equvilent to picks per minute (PPM)
    Computation Time (s)- How long does the policy take to compute (lower is better)
    Execution Time (s) - How long does the policy take to execute (you cannot control this but its important to track)
    Success Rate (%) - What percent of actions results in positive reward 

    In order to calculate these metrics, intially a specialized class was made, but I realised that each metric is essential the same
    mean, std, min, max, etc. calculation, so there seem to be opporunity for better DRY and more generic implementation.

    You may also want to add thresholding checks, if the value is below or above, it can trigger an alert. Which then made me see
    metrics as very related to /diagonistics which I have been thinking about for a bit.

    So I asked Chat "I am thinking of unifying the testing I do on ROS package the the /diagonsitics it publishes, it seems these are very related "

    Who generally seems to agree that metrics and diagnostics are very related, and that the two should be unified.

## How can this be implemented?

    - Seperate the calculation of metric values (happen internally inside node), from the calculation of metric summary statistics and thresholds (happens in this class)
    from the publishing of the metrics (may or may not happen depending on the node)
    - Calculating metrics of summary statics is very cheap, the actual metrics may be more expensive, so If you only want to do that occasional leave it up to the node
    potetially in the future I could add some helpers for that in this metric class but seem low value
    - 3 Levels of testing
        - Unit Test - call the metric/diagonistic function directly (avoid RCLPY overhead)
        - Integration testing - subscribe to /diagonistic
        - System testing - aggregate /diagonistics from all the nodes
    - During testing the DiagonisticArray with levels OK, WARN, ERROR is standard interface used to determine if the test passes of fails.
    - [KEY IDEA] Instead of duplicating logic in the test of what good looks like, I should put it within the node! So that I can test it during testing, but also during deployment if I choose.
    that is the key idea! If its worth testing at test time, its likely worth monitoring in production. By use diagonistics, we can get both at the same time.
    - How would this apply to reward for example? I have a mock env that I am expecting to return a reward value of 19. 
    First of all, the diagonistic should throw an error if the reward is away from an expected value, so I can set the excepted value to 19, run the env, and check the metric. Ok makes sense!
    - Key idea is to define what is working once, and then reuse it everywhere: in the UI, in monitoring, and in tests.
    - It happens in a distributed fashion, each node can define what working means for itself. Encapsulates the logic of what working means for a node.
    - Diagonistics should be minimal and complete. If everything is OK, then you can sleep at night. If you are still uncertain, you need another test!
    - Test Driven Architecture that unifies development, testing and continual monitoring. When you except an OK in pytest, you should execpt and OK in diagonistics.
    - Why would you want an OK in pytest but not in diagonistics? Potetially it could be too low level? Option to set level just like INFO, DEBUG, WARN, logging
    - Diagonistic class is in charge of putting metrics in DiagonisticArray msg and publishing it to /diagnostics
    - Integration tests done through unified method of subscribing to /diagnostics and reading the values


    ### Chats Feedback
    Chat:
        Push this harder: Metrics → Diagnostics → Assertions.

    Me: 
    
        I think this is whats going on
        FR -> Metrics -> Diagnostics -> Assertion (used in Testing & Monitoring)
        
        - I should make a rule that assertions just happen on OK, WARN, ERROR. the node should internally manager its thresholds.

        # Example 1: Performance Monitoring
        Fast pickup -> loop_time (s) -> Warn if 1 < loop_time <= 1.5, Error if loop_time > 1.5 -> time.sleep(2) Assert ERROR or Assert OK
        
        # Example 2: Success Rate Monitoring  
        Successful pickup -> success_rate (%) -> Warn if 80 < success_rate <= 90, Error if success_rate <= 80 -> Assert success_rate > 85
        
        # Example 3: Resource Usage
        Memory usage -> memory_usage (MB) -> Warn if 512 < memory <= 768, Error if memory > 768 -> Assert memory < 512
        
        # Example 4: Reward Validation
        Expected reward -> reward_value -> Warn if |reward - expected| > 0.1, Error if |reward - expected| > 1.0 -> Assert reward == expected
        
        # Example 5: Action Distribution
        Exploration vs exploitation -> action_entropy -> Warn if entropy < 0.5, Error if entropy < 0.1 -> Assert entropy > 0.3

    Chat:
        Diagnostics are only as good as the thresholds they encode. If you make them the arbiter of correctness, they need to be:

        Well-calibrated (e.g. WARN if < 80%, ERROR if < 50%) and Context-aware (e.g. allow slower latency in sim vs real)

        Diagnostics encode your current belief about correctness, based on metrics and thresholds. They are a testable, observable
        and system-wide way to answer "is it working?" — but only as accurate as the assumptions they embed.

    Chat:
        - Risks emerge if diagnostics try to do too much
        - If too noisy: developers ignore them (false WARNs)
        - If too vague: tests can't rely on them
        - If too specific: they hardcode test assumptions and break


    Chat:
        - Metrics should be computed by the node, since they know context.
        - Diagnostics class should track and summarize them, possibly lazily (cool idea, no need to do every time only when request, once a day?)
        - Diagnostics should be treated as an interface — a contract exposed by each node. If no one listens, the contract is silent. 
        But if everything is built around it (tests, UI, monitoring), you get modular, observable, testable components.



## References:

    + See Christian Henkels Talk: 
        https://roscon.ros.org/2024/talks/How_is_my_robot_-_On_the_state_of_ROS_Diagnostics.pdf
        https://vimeo.com/1024970271

        - Diaognistics is like a control panel at a power station, or dash board on the car. It provides a snapshot of the current state of the system. Green, Amber, Red lights.
        - Used by humans (developers, operators, repair technicians)
        - Each status has a state [OK, WARN, ERROR], a text summary and additional key-value pairs
        - There are premade diagonistics features for CPU, RAM, etc.
        - Each node can publish its own diagnostics, and the system will aggregate them and display them in the UI.

        Good Diagonistics:
        - About observring current state of robot
        - Try to limit metrics to <10 ideally 2-3 per component
        - Warnings are states that are unusal but allow continued operation
        - Error states indicate the robot should be stopped immedietly and cannot continue operating
        - vs Logging: captures inner state of system, diagonistics is more about an outer state API
        - vs Bagfiles: useful to record diagonistics inside of a bagfile with other system info
        - vs Testing: can help localize issues but don't replace testing, crucial diagonistics may be tested them selves

        Bad Diagonistics:
        - To much red: its light a highlighter. When its red it should really mean a problem, otherwise we get use to it and it becomes meaningless. same applies for WARN
        - Diaognistics are not meant to be used functionally. The end point should be a humans eye, or computer verification, they are not service calls that should be acted upon.
        - Diagonistics must be recivied. If no one is tracking them they are useless! (my idea here is to make the computer track them via metrics class)


    + Autoware which has some great docs on testing: https://autowarefoundation.github.io/autoware-documentation/main/contributing/testing-guidelines/

        They then have a particular section, where they test based on the diagonistics values:

        https://tier4.github.io/driving_log_replayer/
        https://tier4.github.io/driving_log_replayer/use_case/performance_diag/

        Driving Log Replayer is a ROS 2 package that evaluates the performance of Autoware.Universe based on previously recorded input data. 
        It allows for defining and checking whether the assumptions of the system operation have been met.

        The system reads the rosbag and outputs diagonistics. The evaluator node subscribes to `/diagnostics` and checks if the values are within the expected range.

        For example:

            For blockage evaluation, acquire data with LiDAR intentionally covered with a material that will not pass the laser beam (for example box). 
            The evaluation confirms that ERROR for blockage is output more than a certain rate for the considered situation. 
            The node will also confirm that no ERROR is generated for not covered LiDAR.

        I like how they test both the true and false case (TP, TN, FP, FN)



    + ROS2 Control has config to set thresholds for diagnostics:

        - I like how they set warn and error thresholds

        https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#:~:text=diagnostics

        ``` YAML
            diagnostics:
            threshold:
                controller_manager:
                periodicity:
                    mean_error:
                    error: 10.0
                    warn: 5.0
                    standard_deviation:
                    error: 10.0
                    warn: 5.0
        ``` 


Risk 1: Don't want to involve RCLPY in unit testing
CM 1: Allow direct access to metric trackers. Decouple metric tracking and diagonistics publishing.



Risk 2:Testing has one purpose and diagonistics another, if you try to do to many things you can clutter both. The purpose of diagonistics is to provide high level
information about a system, while testing takes a variety of forms, and may not want to be constrained by having to be unified. Diagonistics for example, may want
to be faster and lightweight, while testing may be more comprehensive. Ie. testing scenarios in gazebo, etc., this doesn't really fall into the diagonistics category,

I guess diagonistics are how you could determine if its right or wrong. Diagonistics are a part of testing!

CM 2: Diagonistics unify the logic of correctness, testing wraps around ontop of it!


TLDR:

- The big difference is that instead of setting thresholds and doing assertions in the test file, you the "health" logic in inside the node, calculated
in the introspect() function along with any viz. and then in the test file you just check OK, WARN, ERROR. Ideally you don't need to check the actual key values either,
all the logic is encapsulated in the node, and the test file just checks the status.


### Thresholds to set

Previously I had set mean_error like ros2_control but it was a bit confusing what that meant! as it required a target
while the other metrics didn't. I think it is cleaner too define a two sided abs value

- drift too far from a nominal value (e.g. mean_min and mean_max)
- exhibit instability (std)
- spike (max) or drop (min) to suspicious extremes

## Future metrics could be:
- delta_mean to track rate of change
- outlier count, some failures are ok, alot are not!
- a mean is exactly that, its a probailistic interpertation of noisey data...
- frequency? that is a higher level metric that you then calculate the stastics for
- consider adding median, or counting number of nans.
- Basically we are just looking at basic summary statistics, similar to a box chart

- You can plot the timeseries of the key,values to see how they change over time to detect drift in memory usage for example, etc.


This is a class that can be included as low level as the GenericServiceClient or as high as the GenericAgent

It basically sets up metric tracking, triggers, and creates diagonistics messages


The node/class that uses the DiagnosticTracker should be able to register metrics, calculate the metrics, and append the values.

The DiagonisticTracker should deal with the thresholds, etc. 

I don't want the class to have to worry about the config for loading the thresholds etc.

Ok this is inspired by ROS2 Control, where you pass the controller manager a config file for the controllers, and then 
you load the controllers. I am not sure if its the best design, its a bit finciky to use, but It seems like it should work.

Should you have the option to not set a threshold? What if I don't know what a good inital value would be?
As Massimo says "its better to have a bad threshold than no threshold", ok, it will throw an error if you havent' set it then.

When you register a metric, you can override the yaml values.

