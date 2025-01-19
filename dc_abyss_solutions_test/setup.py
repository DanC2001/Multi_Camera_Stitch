from setuptools import find_packages, setup

package_name = 'dc_abyss_solutions_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_camera_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Cook',
    maintainer_email='cookdaniel98@gmail.comn',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_node = dc_abyss_solutions_test.multi_camera_node:main',
        ],
    },
)
