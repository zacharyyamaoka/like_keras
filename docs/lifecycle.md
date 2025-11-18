### References:

https://design.ros2.org/articles/node_lifecycle.html



> **Primary Lifecycle States:**
>
> 1. **Unconfigured**
> 2. **Inactive**
> 3. **Active**
> 4. **Finalized**
>
> *Transitions between these primary states typically require action from an external supervisory process, except when an error occurs in the Active state.*
>
> ---
>
> **Transition (Intermediate) States:**
>
> - **Configuring**
> - **CleaningUp**
> - **ShuttingDown**
> - **Activating**
> - **Deactivating**
> - **ErrorProcessing**
>
> *These states occur during lifecycle transitions. Logic in these states determines if the requested transition is successful. The result (success or failure) is communicated back to lifecycle management software via a dedicated interface.*
>
> ---
>
> **Transitions Exposed to Supervisory Processes:**
>
> - `create`
> - `configure`
> - `cleanup`
> - `activate`
> - `deactivate`
> - `shutdown`
> - `destroy`



- https://docs.ros2.org/foxy/api/rclpy/api/node.html
    - Has destory_node()
- https://docs.ros.org/en/rolling/p/rclpy/rclpy.lifecycle.html
    - LifecycleNode(LifecycleNodeMixin, Node)
- [Ark Robotics](https://github.com/Robotics-Ark/ark_framework/blob/main/ark/client/comm_infrastructure/comm_endpoint.py)
    - suspen_commucation(), resume_communciations(), kill_node(), suspen_node(), retart_node()
- [Unity Hooks](https://docs.unity3d.com/6000.2/Documentation/ScriptReference/MonoBehaviour.Awake.html)
    - Awake(), Start(), Update(), OnDsiable(), on Distroy()



- Essentially its a small state machine, with transitions functions

