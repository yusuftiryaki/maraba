from setuptools import find_packages, setup

package_name = "robot_kit"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enes",
    maintainer_email="enesis@entes.com.tr",
    description="The kit socket: the orchadOS scan kit plugged into the robot (docs/kit-soketi.md).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kit_manager = robot_kit.kit_manager_node:main",
            "kit_gnss = robot_kit.kit_gnss_node:main",
            "gps_selector = robot_kit.gps_selector_node:main",
            "ntrip_client = robot_kit.ntrip_client_node:main",
            "fake_kit = robot_kit.fake_kit:main",
            "kit_camera = robot_kit.kit_camera_node:main",
            "fake_camera_shim = robot_kit.fake_camera_shim:main",
            "trunk_localizer = robot_kit.trunk_localizer_node:main",
        ],
    },
)
