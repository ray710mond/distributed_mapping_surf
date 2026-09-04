from pathlib import Path


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'slam_to_scanlock_supervisor.py'


def test_supervisor_has_no_pcd_generation_writer():
    supervisor = SCRIPT.read_text()

    assert 'save_pcd_xyz' not in supervisor
    assert 'generation_' not in supervisor


def test_launch_combines_bounds_and_overlap_for_handoff():
    launch = (Path(__file__).parents[1] / 'launch' / 'hardware.launch.py').read_text()

    assert "'initial_reference_available': localization_enabled" in launch
    assert "'initial_reference_available': localization_enabled" in launch
    assert 'critical_mass.' not in launch
    assert "'handoff.exit_overlap'" in launch
    assert "'handoff.consecutive_exit_updates'" in launch
    assert "'bounds_margin'" in launch

    supervisor = SCRIPT.read_text()
    assert "self.state = 'FASTLIO_GLOBAL'" in supervisor
    assert 'self.tf_broadcaster.sendTransform(transform)' in supervisor
    assert 'self.last_good_correction' in supervisor
    assert 'self.inside_bounds and' in supervisor
