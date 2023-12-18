from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autorace_core_CVlization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'signs'), glob(os.path.join('signs', '*.png'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), ['config/compensation.yaml', 'config/projection.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pritor',
    maintainer_email='petya.podstavkin@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_finder=autorace_core_CVlization.lane_finder:main',
            'lane_follower=autorace_core_CVlization.lane_follower:main',
            'image_projector=autorace_core_CVlization.image_projector:main',
            'image_compensation=autorace_core_CVlization.image_compensation:main',
            'state_decider= autorace_core_CVlization.state_decider:main',
            'signs_analyzer=autorace_core_CVlization.signs_analyzer:main',
            'intersection=autorace_core_CVlization.intersection:main',
            'construction=autorace_core_CVlization.construction:main',
            'parking=autorace_core_CVlization.parking:main',
            'pedestrian=autorace_core_CVlization.pedestrian:main',
            'tunnel=autorace_core_CVlization.tunnel:main'
        ],
    },
)
