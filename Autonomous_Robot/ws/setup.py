import os
from glob import glob

data_files=[
    	# Install marker file in the package index
    	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    	# Include our package.xml file
    	(os.path.join('share', package_name), ['package.xml']),
    	# Include all launch files, last 2 added manually
    	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    	(os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.urdf'))),
    	(os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz')))
	],
	install_requires=['setuptools'],