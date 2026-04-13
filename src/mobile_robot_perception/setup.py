from setuptools import find_packages, setup

package_name = 'mobile_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'box_detector = mobile_robot_perception.box_detector:main' ,
            'angle_estimator = mobile_robot_perception.angle_estimator:main',
            'object_localizer = mobile_robot_perception.object_localizer:main',
            'target_manager = mobile_robot_perception.target_manager:main' ,
        ],
    },
)
