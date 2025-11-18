# bam_diagonistics
---

Package for diagonistics tools shared accross workspace:

- default diagonistic threshold configs
- common DiagonisticTask types

- including it as a package in bam_ros, as its deeply integrated with system

## Design Notes




## Patches

### Parameters declared multiple times
---
With the current design of making a diagonistics updater in each component, you have an issue beacuse each one will look for the same period parameter.

```
rclpy.exceptions.ParameterAlreadyDeclaredException: Parameter(s) already declared: ['diagnostic_updater.period']
```

Patch is to add a check 
```
        if not self.node.has_parameter('diagnostic_updater.period'):
            self.node.declare_parameter('diagnostic_updater.period', 1.0)

        self.__period = self.node.get_parameter('diagnostic_updater.period').value
```

Its assumed you would just have one diagonistics per node I guess

#TODO would be to multiplex the same publisher? Mabye its better to have different publiers onto the same topic though...

Yuck it hard to patch this... beacuse the package is already a dependcy for realsense2-camera...

```
The following packages will be REMOVED:
  ros-jazzy-diagnostic-updater ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-dbgsym
```

Its a pain to edit the file though... Ok the change is so small fine, I will patch it? its a hack fix but for now ok.
