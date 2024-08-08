from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_av'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the resource directory
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamid_ubuntu',
    maintainer_email='hrhhemati@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = simple_av.control:main',
            'localization = simple_av.localization:main',
            'planning = simple_av.planning:main',
            'perception = simple_av.perception:main'
        ],
    },
)
