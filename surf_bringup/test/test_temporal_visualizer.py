"""Exercise the production visualization callback without starting ROS nodes."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from surf_multirobot_msgs.msg import VoxelDelta


def test_spatial_priority_and_deleted_timestamp_are_preserved():
    path = Path(__file__).parents[1] / 'scripts' / 'slam_map_visualizer.py'
    spec = importlib.util.spec_from_file_location('visualizer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    view = SimpleNamespace(voxels={}, observation_times={}, retired_epochs=set(),
                           map_epoch=None, version=0, _publish_voxels=lambda stamp: None)

    def apply(x, stamp, state, version, epoch=1):
        m = VoxelDelta()
        m.operating_mode = 4
        m.map_epoch = epoch
        m.version = version
        m.resolution = .05
        m.x = [x]
        m.y = [0]
        m.z = [0]
        m.state = [state]
        m.observation_time_ns = [stamp]
        module.SlamMapVisualizer._delta_callback(view, m)

    apply(1, 105, 4, 10)
    apply(2, 90, 1, 2)
    apply(1, 100, 1, 11)
    assert (1, 0, 0) not in view.voxels
    assert (2, 0, 0) in view.voxels
    assert view.observation_times[(1, 0, 0)] == 105
    apply(1, 1, 1, 1, epoch=2)
    apply(2, 200, 1, 15, epoch=1)
    assert view.map_epoch == 2
    assert (2, 0, 0) not in view.voxels
