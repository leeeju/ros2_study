from setuptools import find_packages, setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',[
            'launch/path_planning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='null',
    maintainer_email='02stu4@gmail.com',
    description='ROS2 path planning package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning = path_planning.path_planning:main',
            'path_planning2 = path_planning.path_planning_2:main'
        ],
    },
)
