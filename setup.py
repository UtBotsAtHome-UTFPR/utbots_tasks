from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'utbots_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['utbots_tasks', 'utbots_tasks.*'], exclude=['test']),
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
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'recognition = utbots_tasks.states.basic_face:main',
            'basic_vision = utbots_tasks.states.basic_vision:main',
            'basic_nav = utbots_tasks.states.basic_nav:main',
            'basic_voice = utbots_tasks.states.basic_voice:main',
            'voice_nav_smach = utbots_tasks.states.voice_nav:main',
            'take_pictures = utbots_tasks.states.take_pictures:main',
            'concurrence = utbots_tasks.states.concurrence_test:main',
            'inspection = utbots_tasks.tasks.inspection:main',
            'person_recognition = utbots_tasks.tasks.person_recognition:main',
            'manipulation_object_detection = utbots_tasks.tasks.manipulation_and_object_detection:main',
            'receptionist = utbots_tasks.tasks.receptionist:main',
            'recep_calouros = utbots_tasks.tasks.recep_calouros:main',
            'follow_me = utbots_tasks.tasks.follow_me:main',
            'carry_my_luggage = utbots_tasks.tasks.carry_my_luggage:main',
            'beverage_search = utbots_tasks.tasks.beverage_search:main',
            'qa = utbots_tasks.tasks.qa:main'
        ],
    },
)