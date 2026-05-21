from setuptools import setup

package_name = "px4_rerun_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        #        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "rerun-sdk"],
    zip_safe=True,
    maintainer="Kevin",
    maintainer_email="kevin@nuclea.solutions",
    description="Bridge from px4_control topics to the Rerun viewer.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rerun_bridge = px4_rerun_bridge.bridge:main",
        ],
    },
)
