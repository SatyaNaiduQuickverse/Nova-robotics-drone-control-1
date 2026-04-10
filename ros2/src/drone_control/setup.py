from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NovaROS',
    maintainer_email='novaros@local',
    description='Safety-critical drone control via MAVROS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = drone_control.control_node:main',
        ],
    },
)
