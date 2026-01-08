from setuptools import setup
from glob import glob
import os

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # URDF/XACRO files
        (os.path.join('share', package_name, 'urdf', 'xacros'), 
            glob('urdf/xacros/*.xacro')),
        # Mesh files (both .stl and .STL)
        # Mesh files (both .stl and .STL)
        # Mesh files (both .stl and .STL)
        (os.path.join('share', package_name, 'urdf', 'meshes'), 
            glob('urdf/meshes/*.stl') + glob('urdf/meshes/*.STL')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config', 'slam'),
            glob('config/slam/*.yaml')),
        (os.path.join('share', package_name, 'config', 'nav2'),
            glob('config/nav2/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My robot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)