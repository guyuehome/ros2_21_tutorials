from setuptools import setup

package_name = 'learning_action'

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
    maintainer='hcx',
    maintainer_email='hcx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'action_move_client    = learning_action.action_move_client:main',
         'action_move_server    = learning_action.action_move_server:main',
        ],
    },
)
