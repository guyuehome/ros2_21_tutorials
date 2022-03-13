from setuptools import setup

package_name = 'learning_topic'

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
         'topic_helloworld_pub  = learning_topic.topic_helloworld_pub:main',
         'topic_helloworld_sub  = learning_topic.topic_helloworld_sub:main',
         'topic_webcam_pub      = learning_topic.topic_webcam_pub:main',
         'topic_webcam_sub      = learning_topic.topic_webcam_sub:main',
         'interface_object_pub  = learning_topic.interface_object_pub:main',
         'interface_object_sub  = learning_topic.interface_object_sub:main',
        ],
    },
)
