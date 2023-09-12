from setuptools import find_packages, setup

package_name = 'spidar_sdk'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spidar_sdk.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leeeju',
    maintainer_email='02stu4@gmail.com',
    description='spidar_sdk_node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spidar_sdk_node = spidar_sdk.spidar_sdk_node:main"
        ],
    },
)
