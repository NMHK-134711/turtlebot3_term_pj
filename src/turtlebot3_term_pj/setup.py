from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlebot3_term_pj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['turtlebot3_term_pj', 'turtlebot3_term_pj.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/action', glob('action/*.action')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hk',
    maintainer_email='hk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = turtlebot3_term_pj.patrol_node:main',
            'ui_main_node = turtlebot3_term_pj.main:main',
            'teleop_gui = turtlebot3_term_pj.teleop_gui:main',
            'master = turtlebot3_term_pj.master:main',
            'fake_turn_node = turtlebot3_term_pj.fake_turn_node:main',
            'fake_yolo_node = turtlebot3_term_pj.fake_yolo_node:main'
        ],
    },
)
