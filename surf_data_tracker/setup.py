from setuptools import find_packages, setup

package_name = 'surf_data_tracker'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/examples', [
            'examples/summary.csv', 'examples/timeseries.csv']),
        ('share/' + package_name + '/launch', ['launch/tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
            'halow_capacity_provider = surf_data_tracker.capacity:main',
            'identify_allocation = surf_data_tracker.identification:main',
        'data_tracker = surf_data_tracker.tracker:main',
        'analyze_run = surf_data_tracker.analysis:main',
        'collect_run = surf_data_tracker.collect:main',
        'merge_hardware_runs = surf_data_tracker.merge_hardware_runs:main',
        'clock-sync-status = surf_data_tracker.clock:main',
    ]},
)
