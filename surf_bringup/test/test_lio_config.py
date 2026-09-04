from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / 'config'
LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def _parameters(filename):
    with (CONFIG_DIR / filename).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['/**']['ros__parameters']


def test_drone_mid360_keeps_calibrated_extrinsic_fixed():
    params = _parameters('drone_mid360.yaml')

    assert params['mapping']['extrinsic_est_en'] is False


def test_mid360_base_config_supplies_rigid_extrinsic():
    params = _parameters('mid360_lio.yaml')
    translation = params['mapping']['extrinsic_T']
    rotation = params['mapping']['extrinsic_R']

    assert len(translation) == 3
    assert len(rotation) == 9


def test_hardware_launch_is_mid360_only():
    hardware_launch = (
        LAUNCH_DIR / 'hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "package='livox_ros_driver2'" in hardware_launch
    assert "'common.imu_topic': 'livox/imu'" in hardware_launch
    assert "LaunchConfiguration('sensor')" not in hardware_launch


def test_hybrid_policy_always_runs_fastlio_and_monitors_overlap():
    hardware_launch = (
        LAUNCH_DIR / 'hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "executable='fastlio_mapping'" in hardware_launch
    assert "executable='localization_overlap_monitor'" in hardware_launch
    assert "'reference_pcd': localization_map" in hardware_launch
    assert "get_package_share_directory('surf_slam')" not in hardware_launch
    assert "'initial_reference_available': localization_enabled" in hardware_launch
    assert "'target_frame': mapping_frame" in hardware_launch
    assert "'fallback_to_input_frame': not localization_enabled" in hardware_launch
    assert "executable='slam_to_scanlock_supervisor'" in hardware_launch
    assert "'scan_lock_executable': _package_executable(" in hardware_launch


def test_shutdown_map_save_is_a_launch_argument():
    hardware_launch = (
        LAUNCH_DIR / 'hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "'save_map_on_shutdown'" in hardware_launch
    assert (
        "'map_storage.save_on_shutdown': save_map_on_shutdown"
        in hardware_launch)
    assert "'map_storage.save_on_shutdown': True" not in hardware_launch


def test_hardware_can_skip_startup_map_conversion():
    hardware_launch = (
        LAUNCH_DIR / 'hardware.launch.py'
    ).read_text(encoding='utf-8')
    assert "'prepare_map_on_startup'" in hardware_launch
    assert 'prepare_map_on_startup is false' in hardware_launch


def test_reference_grid_is_published_on_global_map_topic():
    hardware_launch = (
        LAUNCH_DIR / 'hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "'topic_name': '/map/reference'" in hardware_launch
    assert "executable='live_occupancy_projector'" in hardware_launch
    assert "'output_topic': '/map'" in hardware_launch


def test_laptop_launch_runs_humanoid_bringup_and_rviz():
    laptop_launch = (
        LAUNCH_DIR / 'laptop_humanoid_rviz.launch.py'
    ).read_text(encoding='utf-8')

    assert "'role': 'humanoid'" in laptop_launch
    assert "'launch_rviz': 'true'" in laptop_launch
    assert "'livox_config': runtime_path" in laptop_launch
    assert "'livox_host_ip'" in laptop_launch
    assert "'livox_lidar_ip'" in laptop_launch

    rviz = (
        Path(__file__).parents[1] / 'rviz' / 'humanoid_map.rviz'
    ).read_text(encoding='utf-8')
    assert '/humanoid/transport/initialpose' in rviz
    assert '/humanoid/transport/drone_initialpose' in rviz
    assert '/laptop/transport/' not in rviz


def test_role_configs_do_not_hardcode_robot_namespaces():
    for filename in ('drone_mid360.yaml', 'humanoid_mid360.yaml'):
        common = _parameters(filename)['common']
        assert 'lid_topic' not in common
        assert 'imu_topic' not in common
        assert 'odom_frame' not in common
        assert 'imu_frame' not in common
