from setuptools import find_packages, setup

package_name = 'turtle_project'

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
    maintainer='georgeu',
    maintainer_email='georgeu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "t_c_exe = turtle_project.turtle_controller:main",
            "t_s_exe = turtle_project.turtle_spawner:main"
        ],
    },
)
