from setuptools import setup

package_name = 'mtlt305_ros'

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
    maintainer='kernt',
    maintainer_email='tobiasbenjamin.kern@carissma.eu',
    description='ros wrapper for Aceinna\'s MTLT305 tilt sensor and IMU',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtlt305_node=mtlt305_ros.MTLT305_node:main',
        ],
    },
)
