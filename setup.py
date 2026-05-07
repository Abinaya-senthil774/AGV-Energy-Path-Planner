from setuptools import find_packages
from setuptools import setup

package_name = 'moiro_agv_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # To be added
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'moiro_agv_interactive_marker.launch.py'))),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'moiro_agv_obstacle_detection.launch.py'))),
        # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz',
        #                                               'moiro_agv_interactive_marker.rviz'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Ryan Shim', 'Gilbert'],
    author_email=['jhshim@robotis.com', 'kkjong@robotis.com'],
    maintainer='Will Son',
    maintainer_email='willson@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of Different moiro_agv Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'energy_planner_node = moiro_agv_example.energy_path_planner.energy_planner_node:main',
            'agv1_node = moiro_agv_example.agv_nodes.agv1_node:main',
            'agv2_node = moiro_agv_example.agv_nodes.agv2_node:main',
            'task_dispatcher_node = moiro_agv_example.dispatcher.task_dispatcher_node:main',
       
            # To be added
            # 'moiro_agv_interactive_marker = \
            #   moiro_agv_example.moiro_agv_interactive_marker.main:main',
            #'moiro_agv_obstacle_detection = \
            #    moiro_agv_example.moiro_agv_obstacle_detection.main:main',
            #'moiro_agv_patrol_client = \
            #    moiro_agv_example.moiro_agv_patrol_client.main:main',
            #'moiro_agv_patrol_server = \
            #    moiro_agv_example.moiro_agv_patrol_server.main:main',
            #'moiro_agv_position_control = \
            #    moiro_agv_example.moiro_agv_position_control.main:main',
        ],
    },
)
