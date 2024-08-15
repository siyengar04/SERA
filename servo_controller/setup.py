from setuptools import setup
import os
from glob import glob

package_name = "servo_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "description"),
            glob("description/*.xacro"),
        ),
        # Add other directories with resources as needed
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="Description of your package",
    license="License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_controller = servo_controller.servo_controller:main",
            "controller_to_servo = servo_controller.controller_to_servo:main",
            "robotcontroller = servo_controller.robotcontroller:main",
            "controller_to_servo_n = servo_controller.controller_to_servo_n:main",
            "joystick_to_joint_control = servo_controller.joystick_to_joint_control:main",
        ],
    },
)
