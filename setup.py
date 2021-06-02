from setuptools import setup

package_name = 'udemy_ros2_pkg'

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
    maintainer='callum',
    maintainer_email='xdgfx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publisher = udemy_ros2_pkg.publisher:main",
            "subscriber = udemy_ros2_pkg.subscriber:main",
            "measure_wheel_speed = udemy_ros2_pkg.measure_wheel_speed:main",
            "calculate_robot_speed = udemy_ros2_pkg.calculate_robot_speed:main",
        ],
    },
)
