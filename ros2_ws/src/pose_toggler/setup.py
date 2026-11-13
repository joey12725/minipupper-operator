from setuptools import find_packages, setup

package_name = 'pose_toggler'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joey Hersh',
    maintainer_email='joeyandpaige@gmail.com',
    description='Cycles a Mini Pupper through joint-space pose presets.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_toggler = pose_toggler.pose_toggler:main',
        ],
    },
)
