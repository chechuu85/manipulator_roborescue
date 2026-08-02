import os
from glob import glob
from setuptools import setup, find_packages
from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch_scripts', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.urdf*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesusaguilar',
    maintainer_email='jesusaguilarromero01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
