from setuptools import find_packages, setup

package_name = 'barracuda_thrusters'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['t200_18v_data.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihir',
    maintainer_email='mihirsin@usc.edu',
    description='barracuda_thrusters package converts desired force values for each thruster (which it receives from the control module) into values that can be output on the PWM pins on the microcontrollers on the thruster boards, then sends the appropriate values to the appropriate registers on the microcontrollers via serial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barracuda_thrusters = barracuda_thrusters.barracuda_thrusters:main'
        ],
    },
)
