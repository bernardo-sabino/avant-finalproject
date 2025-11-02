from setuptools import find_packages, setup

package_name = 'mavros_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
       ('share/ament_index/resource_index/packages', ['resource/mavros_test']),
       ('share/mavros_test', ['package.xml']),
       ('share/mavros_test/launch', ['launch/mavros_udp_launch.py']),
      ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
     },
    entry_points={
        'console_scripts': [
        ],
    },
)
