from setuptools import setup
package_name = 'bottle_align_action'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimtaehee',
    maintainer_email='kimtaehee@todo.todo',
    description='Bottle alignment action package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'align_bottle_server_v3 = bottle_align_action.align_bottle_server_v3:main',
            'bottle_align_topic_node = bottle_align_action.bottle_align_topic_node:main',
        ],
    },
)
