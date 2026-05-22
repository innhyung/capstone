from setuptools import find_packages, setup

package_name = 'bno055_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danial',
    maintainer_email='nblue27@gmail.com',
    description='BNO055 IMU driver for capstone mobile robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_node = bno055_driver.bno055_node:main',
        ],
    },
)
