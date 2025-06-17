from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'utbots_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['utbots_tasks', 'utbots_tasks.*'],exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='utbots.home@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={
    'test': ['pytest', 'other-test-deps'],
    },
    entry_points={
        'console_scripts': [
            'beverage_search = utbots_tasks.tasks.beverage_search:main',
            'recognition = utbots_tasks.states.basic_face:main',
            'voice_nav_smach = utbots_tasks.tasks.voice_nav:main',
        ],
    },
)
