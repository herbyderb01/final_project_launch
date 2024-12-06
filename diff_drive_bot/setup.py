from setuptools import setup
import os
from glob import glob

package_name = 'diff_drive_bot'

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
    description='Final Project launch',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_tf_broadcaster = diff_drive_bot.pose_to_tf_broadcaster:main',
        ],
    },
)
# from setuptools import setup
# import os
# from glob import glob

# package_name = 'diff_drive_bot'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         # Required by ament_index
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # Additional directories for ROS-specific data
#         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
#         (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
#         (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
#         (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
#         (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
#         (os.path.join('share', package_name, 'description'), glob('description/*.*')),
#         (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
#     ],
#     install_requires=['setuptools', 'shapely'],
#     zip_safe=True,
#     maintainer='Nate',
#     maintainer_email='A02307138@aggies.usu.edu',
#     description='Final Project launch and robot description files',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'pose_to_tf_broadcaster = diff_drive_bot.pose_to_tf_broadcaster:main',
#         ],
#     },
# )
