from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'trafficsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rosidl_default_runtime'],
    zip_safe=True,
    maintainer='calvinearnshaw',
    maintainer_email='calvin.earnshaw@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train = trafficsim.train:main',
            'scheduler = trafficsim.scheduler:main'
        ],
    },
)
