from setuptools import setup, find_packages

package_name = "pose_toggler"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name]),
    install_requires=[
        "setuptools",   # always good practice
        "rclpy",
        "std_msgs",
    ],
    zip_safe=True,
    maintainer="YOUR NAME",
    license="Apache-2.0",
    description="Cycles Mini Pupper between SIT and STAND",
    entry_points={
        "console_scripts": [
            "pose_toggler = pose_toggler.pose_toggler:main",
        ],
    },
)
