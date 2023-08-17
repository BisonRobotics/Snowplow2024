from setuptools import find_packages, setup

package_name = 'axle_manager'

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
    maintainer='nate',
    maintainer_email='20501643+naterbots@users.noreply.github.com',
    description='Driver for RoboteQ HDC2460',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'hdc2460 = axle_manager.hdc2460_node:main'
        ],
    },
)
