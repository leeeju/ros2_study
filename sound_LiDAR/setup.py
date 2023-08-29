from setuptools import find_packages, setup

package_name = 'sound_LiDAR'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',[
            'launch/sound_LiDAR.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leeeju',
    maintainer_email='02stu4@gmail.com',
    description='test RP Lidar',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            str('sound_LiDAR_2 = sound_LiDAR.sound_LiDAR_2:main'),
        ],
    },
)
