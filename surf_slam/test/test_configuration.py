import json
from pathlib import Path

import pytest

from surf_slam.configuration import _COPIED_CONFIGS, render_glim_config


def test_render_glim_config(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    for filename in _COPIED_CONFIGS:
        (source / filename).write_text('{}\n', encoding='utf-8')

    output = Path(render_glim_config(
        source, tmp_path / 'rendered', role='drone',
        imu_topic='livox/imu',
        points_topic='livox/points'))

    ros_config = json.loads(
        (output / 'config_ros.json').read_text(encoding='utf-8'))
    assert ros_config['glim_ros']['map_frame_id'] == 'drone/map'
    assert ros_config['glim_ros']['odom_frame_id'] == 'drone/glim_odom'
    assert ros_config['glim_ros']['base_frame_id'] == 'drone/glim_body'
    assert ros_config['glim_ros']['points_topic'] == 'livox/points'
    assert ros_config['glim_ros']['extension_modules'] == ['librviz_viewer.so']
    global_config = json.loads(
        (output / 'config.json').read_text(encoding='utf-8'))
    assert global_config['global']['config_global_mapping'] == (
        'config_global_mapping_pose_graph.json')


def test_humanoid_uses_mid360_extrinsics(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    for filename in _COPIED_CONFIGS:
        (source / filename).write_text('{}\n', encoding='utf-8')

    output = Path(render_glim_config(
        source, tmp_path / 'rendered', role='humanoid',
        imu_topic='livox/imu', points_topic='livox/points'))

    ros_config = json.loads(
        (output / 'config_ros.json').read_text(encoding='utf-8'))['glim_ros']
    sensor_config = json.loads(
        (output / 'config_sensors.json').read_text(
            encoding='utf-8'))['sensors']
    assert ros_config['imu_frame_id'] == 'humanoid/glim_body'
    assert ros_config['lidar_frame_id'] == 'humanoid/glim_livox_frame'
    assert ros_config['base_frame_id'] == 'humanoid/glim_body'
    assert ros_config['publish_imu2lidar'] is True
    assert sensor_config['T_lidar_imu'] == [
        -0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0]


def test_rejects_unknown_role(tmp_path):
    arguments = {
        'role': 'invalid', 'imu_topic': '/imu', 'points_topic': '/points'}
    with pytest.raises(ValueError):
        render_glim_config(tmp_path, tmp_path / 'out', **arguments)
