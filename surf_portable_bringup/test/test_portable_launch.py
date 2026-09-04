from pathlib import Path


def test_portable_launch_does_not_own_robot_stack():
    launch = (Path(__file__).parents[1] / 'launch' / 'portable.launch.py').read_text()
    for forbidden in ('livox_ros_driver2', 'fast_lio', 'glim', 'scan_lock', 'px4_state_bridge'):
        assert forbidden not in launch


def test_portable_contract_is_parameterized():
    launch = (Path(__file__).parents[1] / 'launch' / 'portable.launch.py').read_text()
    for field in ('pointcloud_topic', 'odometry_topic', 'map_frame', 'body_frame'):
        assert field in launch


def test_existing_transport_odometry_contract_is_preserved():
    launch = (Path(__file__).parents[1] / 'launch' / 'portable.launch.py').read_text()
    assert "f'transport/{role}_odometry'" in launch
