from setuptools import find_packages, setup


PACKAGE_NAME = 'surf_slam'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', ['launch/slam.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Raymond',
    maintainer_email='raymond@todo.invalid',
    description='GLIM SLAM integration and sensor adapters for SURF.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'livox_custom_to_pc2 = surf_slam.livox_custom_to_pc2:main',
        ],
    },
)
