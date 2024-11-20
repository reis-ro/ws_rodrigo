from setuptools import setup

package_name = 'cone_pose_estimator_python'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Cone Pose Estimator using depth data from CARLA simulator',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'cone_pose_estimator_node = cone_pose_estimator_python.cone_pose_estimator_node:main',
        ],
    },
)
