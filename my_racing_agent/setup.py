"""
from setuptools import setup
import os
from glob import glob

package_name = 'my_racing_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('lib', package_name), 
            glob('my_racing_agent/*.pth') + glob('my_racing_agent/*.pkl') + glob('my_racing_agent/*.json')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Wall follower agent',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node.py = my_racing_agent.drive_node:main',
            'only_wall.py = my_racing_agent.only_wall:main',
            'wall_log.py = my_racing_agent.wall_log:main',
            'nn_drive_node.py = my_racing_agent.nn_drive_node:main',  
        ],
    },
)



"""


from setuptools import setup
import os
from glob import glob

package_name = 'my_racing_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS2 package resources
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        
        # Models, scaler, config JSON
        (os.path.join('share', package_name, 'models'),
            glob('my_racing_agent/*.pth') +
            glob('my_racing_agent/*.pkl') +
            glob('my_racing_agent/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Wall follower agent',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node.py = my_racing_agent.drive_node:main',
            'only_wall.py = my_racing_agent.only_wall:main',
            'wall_log.py = my_racing_agent.wall_log:main',
            'nn_drive_node.py = my_racing_agent.nn_drive_node:main',
        ],
    },
)
