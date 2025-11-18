"""
    Validate that visual object message exports remain importable.
"""

# BAM
import bam.msgs.visual_objects as viz


def test_core_visual_objects_can_instantiate():
    base = viz.VisualObject()
    assert isinstance(base.visual_id, viz.VisualId)

    frame = viz.Frame()
    urdf_frame = viz.UrdfFrame()
    lines = viz.LineSegments()
    grid = viz.Grid()
    cloud = viz.PointCloud()
    urdf = viz.Urdf()

    assert frame.visual_id != ""
    assert urdf_frame.visual_id != ""
    assert len(lines.points) >= 1
    assert isinstance(grid, viz.Grid)
    assert len(cloud.points) == len(cloud.colors)
    assert isinstance(urdf.q, list)


def test_marker_imports_from_submodule():
    marker = viz.Marker()
    arrow = viz.Arrow()
    box = viz.Box()
    cylinder = viz.Cylinder()
    sphere = viz.Sphere()
    mesh = viz.Mesh()

    assert marker.visible
    assert arrow.visible
    assert box.visible
    assert cylinder.visible
    assert sphere.visible
    assert mesh.visible


def test_compound_visual_objects_instantiation():
    path = viz.Path()
    rlist = viz.RList()
    score_cloud = viz.ScoreCloud()
    grasp_path = viz.GraspPath()
    parallel = viz.ParallelGripper()
    claw = viz.ClawGripper()
    grasp_geom = viz.GraspGeometry()
    conveyor = viz.Conveyor()

    assert isinstance(path, viz.CompoundVisualObject)
    assert path.children
    assert isinstance(rlist.children, list)
    assert score_cloud.children
    assert grasp_path.children
    assert isinstance(parallel.children, list)
    assert isinstance(claw.children, list)
    assert isinstance(grasp_geom.children, list)
    assert isinstance(conveyor.children, list)


def test_gui_visual_objects_instantiation():
    namespace_gui = viz.NamespaceVisibilityGui()
    playback_gui = viz.PlaybackGui(num_frames=1, frame_callback=lambda _: [])
    pc_gui = viz.PointCloudGui()
    pose_selector = viz.PoseSelectorGui()
    reach_map = viz.ReachMapGui()
    score_gui = viz.ScoreCloudGui()
    urdf_gui = viz.UrdfJointControlGui()

    assert namespace_gui.visible
    assert playback_gui.visible
    assert pc_gui.visible
    assert pose_selector.visible
    assert reach_map.visible
    assert score_gui.visible
    assert urdf_gui.visible


def test_plotting_visual_objects_instantiation():
    series = viz.Series(y=[0.0, 1.0, 2.0])
    line_plot = viz.LinePlot(series=[series], vertical_lines=[viz.VerticalLine(x=0.5)], horizontal_lines=[viz.HorizontalLine(y=0.0)])
    line_chart = viz.LineChart(y=[0.0, 1.0, 2.0])
    histogram = viz.Histogram(values=[0.1, 0.2, 0.3])

    actuator_data = viz.ActuatorTimelineData(
        name="joint1",
        times=[0.0, 0.5, 1.0],
        actual_positions=[0.0, 0.25, 0.5],
        measured_positions=[0.0, 0.2, 0.45],
        velocities=[0.0, 0.5, 0.0],
        commands=["hold", "move", "hold"],
        min_pos=-1.0,
        max_pos=1.0,
    )
    timeline = viz.MoteusActuatorTimeline(actuators=[actuator_data])

    assert line_plot.n_series == 1
    assert line_chart.y != []
    assert histogram.values != []
    assert timeline.n_actuators == 1


