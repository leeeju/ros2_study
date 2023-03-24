from setuptools import setup

package_name = 'quaternions'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leeeju',
    maintainer_email='02stu@cwsfa.co.kr',
    keywords=['ROS'],
    description=' Create a node to convert quaternions to Euler ',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quaternion_to_euler = quaternions.quaternion_to_euler:main',
        ],
    },
)
