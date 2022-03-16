from setuptools import setup
import os
from glob import glob

package_name = 'learning_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hcx',
    maintainer_email='huchunxu@guyuehome.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'static_tf_broadcaster = learning_tf.static_tf_broadcaster:main',
        	'turtle_tf_broadcaster = learning_tf.turtle_tf_broadcaster:main',
        	'tf_listener = learning_tf.tf_listener:main',
        	'turtle_following = learning_tf.turtle_following:main',
        ],
    },
)
