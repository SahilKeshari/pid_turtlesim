from setuptools import find_packages, setup

package_name = 'turtle_sim_project'

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
    maintainer='sahil-22',
    maintainer_email='sahil-22@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlesim_controller = turtle_sim_project.turtle_controller:main",
            "turtlesim_spawner = turtle_sim_project.turtle_spawner:main",
            "turtlesim_relocate = turtle_sim_project.turtle_relocate:main",
            "turtlesim_controller_pid = turtle_sim_project.turtle_controller_pid:main",
            "turtlesim_grid_goal2 = turtle_sim_project.turtle_grid_goal2:main",
            "turtlesim_rotate_goal3 = turtle_sim_project.turtle_rotate_goal3:main",
            "turtlesim_rotate_chase_goal4 = turtle_sim_project.turtle_rotate_chase_goal4:main",
            "turtlesim_rotate_chase_goal5 = turtle_sim_project.turtle_rotate_chase_goal5:main",
            "turtlesim_rotate_chase_goal6 = turtle_sim_project.turtle_rotate_chase_goal6:main",

        ],
    },
)
