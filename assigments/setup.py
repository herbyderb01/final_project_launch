from setuptools import setup
import os
from glob import glob

package_name = 'assignments'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf'))
    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='Nate',
    maintainer_email='A02307138@aggies.usu.edu',
    description='Nates module 5 code completion',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transforms = assignments.tools.transforms:main',
            '03_single = assignments.03_pysim_ros_single:main',
            '03_broken = assignments.03_pysim_ros_broken:main',
            '03_multi = assignments.03_pysim_ros_multi:main',
            '05_pub_pose = assignments.05_pysim_pose_pub:main',
            '06_pub_pose = assignments.06_pysim_pose_pub:main',
            '06_pub_sensor = assignments.06_pysim_sensor_pub:main',
        ],
    },
)
