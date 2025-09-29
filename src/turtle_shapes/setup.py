from setuptools import setup, find_packages
import os
from glob import glob # <--- Import glob

package_name = 'turtle_shapes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        # Install marker file
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files: Use glob to find all files ending in .launch.py in the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='andrew@todo.todo',
    description='Turtlesim project that draws parametric shapes.',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    # --- All executable nodes are correctly registered here! ---
    entry_points={
        'console_scripts': [
            'shape_node = turtle_shapes.shape_node:main',
            'turtle_commander = turtle_shapes.turtle_commander:main',
            'interactive_turtle = turtle_shapes.interactive_turtle:main',
        ],
    },
)