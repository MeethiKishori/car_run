from setuptools import setup
import os
from glob import glob

package_name = 'my_racing_agent'

# --- HELPER FUNCTION FOR RECURSIVE FILE LISTING ---
def list_files_recursively(base_dir):
    """Return list of all file paths inside base_dir, recursively."""
    file_paths = []
    # Ensure the directory exists before walking
    if os.path.isdir(base_dir):
        for root, dirs, files in os.walk(base_dir):
            for filename in files:
                file_paths.append(os.path.join(root, filename))
    return file_paths

# --- RECURSIVE MODEL INSTALLATION (SIMPLIFIED) ---
# Goal: Install all files in 'models/' to 'share/my_racing_agent/models/'
model_data = []
model_root = 'models'

# Check if the models directory exists and recursively list files
if os.path.isdir(model_root):
    # Iterate through each item inside 'models' (e.g., 'MIT_Tunnel', 'walker_racecourse')
    for root, dirs, files in os.walk(model_root):
        # Create the destination path relative to the install root
        # Example: models/MIT_Tunnel/model.sdf -> share/my_racing_agent/models/MIT_Tunnel/model.sdf
        dest_dir = os.path.join('share', package_name, root)
        
        # Add a tuple (destination_folder, list_of_files) for each directory containing files
        if files:
            file_paths = [os.path.join(root, f) for f in files]
            model_data.append((dest_dir, file_paths))

# --- URDF INSTALLATION (RECURSIVE) ---
# If your URDF directory has includes or subfolders, you need recursive globbing.
urdf_files = []
for root, dirs, files in os.walk('urdf'):
    for filename in files:
        # Get the path relative to the package root 'urdf/'
        urdf_files.append(os.path.join(root, filename))

# Destination path for all URDF files
urdf_data = (os.path.join('share', package_name, 'urdf'), urdf_files)


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Standard ROS 2 files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),

        # Meshes (Simple glob, assuming no subfolders)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),

        # URDF files (Now recursive)
        urdf_data,

        # Models (Recursive Installation)
        *model_data,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='F1TENTH ROS 2 Agent for racing algorithms',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = my_racing_agent.drive_node:main',
        ],
    },
)