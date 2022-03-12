from setuptools import setup

package_name = 'learning_service'

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
         'service_adder_client  = learning_service.service_adder_client:main',
         'service_adder_server  = learning_service.service_adder_server:main',
         'service_object_client = learning_service.service_object_client:main',
         'service_object_server = learning_service.service_object_server:main',
        ],
    },
)
