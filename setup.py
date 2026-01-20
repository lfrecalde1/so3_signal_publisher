from setuptools import find_packages, setup

package_name = 'so3_signal_publisher'

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
    maintainer='eagle1',
    maintainer_email='lfrecalde1@espe.edu.ec',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so3_python = so3_signal_publisher.main:main',
            'trpy_python = so3_signal_publisher.main_trpy:main',
        ],
    },
)
