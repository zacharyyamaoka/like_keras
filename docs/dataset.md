


Ok what do I think are the parts of this system?

- Data is generated inside of components. Then we want to log and view now or save until later.
- Foxglove SDK approach is interesting at it unifies both of thoose things...

Diagonistics and Logging are interesting beacuse its always kinda going to a global thing

Is this different than attaching a dataset thing to a specific thing? perhaps if you always just log all the information then its easy to configure what you actually want to save aftewards... Compression prior to uploading can be a smart idea to save on cost...


Writing API:

Encoding:
    - Compression, filtering, etc.
    - Compression can be computaionally intensive
    - "uploading can be computionally intesnive" mabye you want to do async of when you are no longer busy.


Uploading:


Data Location:

    Where is the data at different stages?
    How does the data move between thoose locations
    When does the data move between thoose locations? (automated, agent, etc.)
    In what format does the data move between thoose locations

    - On the Robot RAM
    - On its harddrive
    - On a local remote storage (edge device)
    - On the cloud
    - on an engineers computer

    If we are using a cloud storage provider who are we using?
        - ReductStore
        - InfluxDB
        - S3
        - Google cloud
    Why are we using them?
      - Price
        - Speed of download?

File Structure:
    - How is the data organized when its stored?
    - Mcap, LeRobot Format, other?

Decoding:



Reading Api:

    - What is the API for querying the data?
    - It seems we have moved around an "events" abstraction
        - Its cool how foxglove overlays events ontop of the datastream while some like heex record things as events chuncks...
        - Foxgloves approach is more general

    - Tesla Data Engine, we want to be able to search for specific events or locations, etc
    - If something happens it would be nice to be able to go back to a point in time and see a bit before a bit after, and go through different subsystems and see what was going on. Mabye even replay the databack agian
    - Idea of a datastream, across different files, but we query it based on things that are semnatically meaningful to use, for example episodes (Sarsa transitions, etc.) We want to avoid duplication of data, that is why its better to store it I think as a contious file thing, then cut it up as you want later!!!

- Post Processing of data



References:

https://github.com/google-deepmind/envlogger

Robo DM Datamangement for large robot datasets:
- https://arxiv.org/html/2505.15558v1

LeRobot
- https://github.com/huggingface/lerobot?tab=readme-ov-file#the-lerobotdataset-format
- https://huggingface.co/docs/lerobot/en/lerobot-dataset-v3
- This is a really good reference!!! We should defintly be based around this...

Delete Episodes - Remove specific episodes from a dataset
Split Dataset - Divide a dataset into multiple smaller datasets
Merge Datasets - Combine multiple datasets into one. The datasets must have identical features, and episodes are concatenated in the order specified in repo_ids
Add Features - Add new features to a dataset
Remove Features - Remove features from a dataset

This is a very important idea:
- A core v3 principle is decoupling storage from the user API: data is stored efficiently (few large files), while the public API exposes intuitive episode-level access.
- Episode‑specific views are reconstructed via metadata, not file boundaries.



Ros Bags:

    Super great design article: https://github.com/ros2/design/blob/ros2bags/articles/rosbags.md

    https://docs.ros.org/en/rolling/Tutorials/Advanced/Reading-From-A-Bag-File-Python.html
    https://docs.ros.org/en/galactic/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html


Foxglove:

    https://foxglove.dev/blog/best-practices-for-recording-and-uploading-robotics-data
    https://docs.foxglove.dev/docs/getting-started/python#recording-data-to-a-file
    https://docs.foxglove.dev/docs/data/edge-sites


Reduct Store:

    https://www.reduct.store/blog/database-for-robotics
    https://www.reduct.store/blog/tutorial-store-ros-data
    https://www.reduct.store/blog/tutorials/ros/optimal-image-storage-solutions-for-ros-based-computer-vision

    Reduct Store Agent:

    - Does smart things with spooled file to allow for seamless ram and harddrive
    - Uplaods in chuncks in between data ingestion and uploading

    https://github.com/reductstore/reductstore_agent
    https://www.reduct.store/docs/how-does-it-work

Dora RS

    https://dora-rs.ai/docs/guides/Debugging/logs
    https://dora-rs.ai/docs/guides/Debugging/metrics


Open Telemetry

    https://opentelemetry.io/docs/languages/python/getting-started/


Better Stack

    https://betterstack.com/
    https://betterstack.com/docs/logs/python/#logging-from-python


InfluxDb

    https://docs.ros.org/en/rolling/p/diagnostic_remote_logging/
    https://github.com/influxdata/telegraf
    https://github.com/iwatake2222/ros2_monitor_grafana
    https://discourse.openrobotics.org/t/diagnostic-remote-logging-announcement/43696
    https://github.com/ros/diagnostics/blob/ros2-humble/diagnostic_remote_logging/src/influxdb.cpp



AWS s3
    https://github.com/aws-robotics/rosbag-uploader-ros1

    AWS ROS1 rolling recorder:

    https://github.com/aws-robotics/rosbag-uploader-ros1/blob/master/rosbag_cloud_recorders/src/rolling_recorder/main.cpp

    - rclpyy multithreaded node
    - one thread can continue to create bags
    - the other thread can manage uploading
    - they turn the rosbag into short 30s bags for atomic chunking it seems like.


What does ideal look like?

- I want to be able to from a super simple python API just record exactly what I want,
    - Any time period
    - Any topics
    - At any resolution/interleaving/compression

- You can only save data retrospectively if you have already recorded it!
    - Lets say you drive to a intersection, and realise you missed the stop sign. You don’t just want the data now, but likely a bit before.
    - Lets say the robot has an error. You don’t just want the data from the past, but likey a bit in the future as well!
- There are different things you may want to record. For a unified approach, you should circuarly buffer a subset of data in the system that you potetially want to save. Then you can take a subset of this subset to save to disk. and perhaps a subset of that subset to save to cloud.
- The things you want to record may change over time (al la tesla data engine) the way you record though doesn’t need to change…
- Data will be stored in different locations, on robot, at edge, on cloud (best for long term peristant)

![image.png](attachment:d0ab2e41-0a06-46ad-8eee-ceaba63a4e75:image.png)

![image.png](attachment:6741c307-c9ac-4e55-9297-54f91c1b8e69:image.png)

# References




Specific Implementations:



[Format Documentation](https://www.notion.so/Format-Documentation-23c0d79b0f9080aeaad2e722659705e3?pvs=21)

[Freedom Robotics Docs](https://www.notion.so/Freedom-Robotics-Docs-23c0d79b0f908076b433ef1aafc5e801?pvs=21)

https://arxiv.org/html/2505.15558v1